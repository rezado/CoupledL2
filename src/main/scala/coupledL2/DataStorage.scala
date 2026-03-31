/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 * http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * *************************************************************************************
 */

package coupledL2

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xs.utils.mbist.MbistPipeline
import coupledL2.utils.SplittedSRAM
import xs.utils.debug.HAssert
import xs.utils.cache.common.L2ParamKey
import xs.utils.sram.{SRAMTemplate, SramBroadcastBundle}
import xs.utils.ClockGate

class DSRequest(implicit p: Parameters) extends L2Bundle {
  val way = UInt(wayBits.W)
  val set = UInt(setBits.W)
  val wen = Bool()
}

// mask not used
class DSBeat(implicit p: Parameters) extends L2Bundle {
  val data = UInt((beatBytes * 8).W)
}

class DSBlock(implicit p: Parameters) extends L2Bundle {
  val data = UInt(blockBits.W)
  def := (that: DSECCBankBlock):Unit = {
    if(enableDataECC) {
      val eccDatVec = that.data.asTypeOf(Vec(dataBankSplit, UInt(encBankBits.W)))
      val datVec = eccDatVec.map(_(dataBankBits - 1, 0))
      this.data := Cat(datVec.reverse)
    } else {
      this.data := that.data
    }
  }
  def mergeEccPart(ecc:Vec[UInt]): DSECCBankBlock = {
    val res = Wire(new DSECCBankBlock)
    if(enableDataECC) {
      val datVec = this.data.asTypeOf(Vec(dataBankSplit, UInt(dataBankBits.W)))
      val eccDatVec = VecInit(ecc.zip(datVec).map(e => Cat(e._1, e._2)))
      res.data := eccDatVec.asUInt
    } else {
      res.data := this.data
    }
    res
  }
}

class DSECCBankBlock(implicit p: Parameters) extends L2Bundle {
  val data = if (enableDataECC) {
    UInt((encBankBits * dataBankSplit).W)
  } else {
    UInt((dataBankBits * dataBankSplit).W)
  }
  def eccPart(): Vec[UInt] = {
    if (enableDataECC) {
      val eccDatVec = data.asTypeOf(Vec(dataBankSplit, UInt(encBankBits.W)))
      VecInit(eccDatVec.map(_(encBankBits - 1, dataBankBits)))
    } else {
      VecInit(Seq.fill(dataBankSplit)(0.U(0.W)))
    }
  }
}


class DataStorage(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    // en is the actual r/w valid from mainpipe (last for one cycle)
    // en is used to generate gated_clock for SRAM
    val en = Input(Bool())

    // ECC error
    val error = Output(Bool())

    // 1. there is only 1 read or write request in the same cycle,
    // so only 1 req port is necessary
    // 2. according to the requirement of MCP2, [req.valid, req.bits, wdata]
    // must hold for 2 cycles (unchanged at en and RegNext(en))
    val req = Flipped(ValidIO(new DSRequest))
    val rdata = Output(new DSECCBankBlock)
    val wdata = Input(new DSBlock)
    val dynSets = Input(UInt(64.W))
  })

  // read data is set MultiCycle Path 2
  val array = Module(new SRAMTemplate(
    gen = new DSECCBankBlock,
    set = blocks,
    way = 1,
    singlePort = true,
    latency = 2,
    hasMbist = p(L2ParamKey).hasMbist,
    pipeDepth = 1,
    extraHold = false,
    explicitHold = true,
    suffix = "_l2c_dat"
  ))

  val dynSetBits = Log2(io.dynSets)
  val arrayIdx = Cat(io.req.bits.way, dynSetMask(io.req.bits.set, dynSetBits)(setBits - 1, 0))
  val wen = io.req.valid && io.req.bits.wen
  val ren = io.req.valid && !io.req.bits.wen

  val arrayWrite = Wire(new DSECCBankBlock)
  val arrayWriteData = if (enableDataECC) {
    Cat(VecInit(Seq.tabulate(dataBankSplit)(i =>
      io.wdata.data(dataBankBits * (i + 1) - 1, dataBankBits * i))).map(data => cacheParams.dataCode.encode(data)).reverse)
  } else {
    io.wdata.data
  }
  arrayWrite.data := arrayWriteData

  val arrayRead = array.io.r.resp.data(0)

  // make sure SRAM input signals will not change during the two cycles
  // TODO: This check is done elsewhere
  array.io.w.apply(wen, arrayWrite, arrayIdx, 1.U)
  array.io.r.apply(ren, arrayIdx)

  val error = if (enableDataECC) {
    // cacheParams.dataCode.decode(eccData).error && RegNext(RegNext(io.req.valid && !io.req.bits.wen))
    VecInit(Seq.tabulate(dataBankSplit)(i => arrayRead.data(encBankBits * (i + 1) - 1, encBankBits * i))).
      map(data => cacheParams.dataCode.decode(data).error).reduce(_ | _) && RegNext(RegNext(io.req.valid && !io.req.bits.wen))
  } else {
    false.B
  }

  // for timing, we set this as multicycle path
  // s3 read, s4 pass and s5 to destination
  io.rdata := arrayRead
  io.error := error

  HAssert(!io.en || !RegNext(io.en, false.B),
    "Continuous SRAM req prohibited under MCP2!")

  HAssert(!(RegNext(io.en) && (io.req.asUInt =/= RegNext(io.req.asUInt))),
    s"DataStorage req fails to hold for 2 cycles!")

  HAssert(!(RegNext(io.en && io.req.bits.wen) && (io.wdata.asUInt =/= RegNext(io.wdata.asUInt))),
    s"DataStorage wdata fails to hold for 2 cycles!")
  HAssert.placePipe(2)
}
