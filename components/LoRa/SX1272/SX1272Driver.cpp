/*
 * SX1272Driver.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: emon1
 */

#include "SX1272Driver.h"

SX1272Driver::SX1272Driver() {
	// TODO Auto-generated constructor stub

}

SX1272Driver::~SX1272Driver() {
	// TODO Auto-generated destructor stub
}


void SX1272Driver::SetModem(RadioModems_t modem){
    if(modem == MODEM_FSK){
        FskMode(FSK_OpMode_Sleep);
        LongRangeMode(SX1272_LongRangeMode_FSK);
    }
    else if(modem == MODEM_LORA){
        LoRaOpMode(LoRa_OpMode_SLEEP);
        LongRangeMode(SX1272_LongRangeMode_LoRa);
    }
    settings.Modem = modem;
}

void SX1272Driver::WriteFifo( uint8_t *buffer, uint8_t size )
{
    int i;
    for(i=0;i<size;i++){
        Write(0, buffer[i]);
    }
}

void SX1272Driver::ReadFifo( uint8_t *buffer, uint8_t size )
{
    int i;
    for(i=0;i<size;i++){
        buffer[i] = Read(0);
    }
}

void SX1272Driver::LongRangeMode(SX1272_LongRangeMode_t mode){
	Write( REG_COMMON_OPMODE, mode );
}
void SX1272Driver::Frf(uint32_t freq){
	Write( REG_COMMON_FRFMSB, (freq>>16));
	Write( REG_COMMON_FRFMID, (freq>>8) );
	Write( REG_COMMON_FRFLSB, (freq>>0) );
}
void SX1272Driver::PaConfig(SX1272_PaSelect_t PAsel, uint8_t outputPower){
	uint8_t temp = (PAsel<<SX1272_PaConfig_PaSelect_bp) |
			((outputPower<<SX1272_PaConfig_OutputPower_gp)&SX1272_PaConfig_OutputPower_gm);
	Write( REG_COMMON_PACONFIG,temp );
}
void SX1272Driver::PaRamp(bool LowPnTxPllOff , SX1272_PaRamp_t PaRamp){
	uint8_t temp = (LowPnTxPllOff<<SX1272_PaRamp_LowPnTxPllOff_bp )|
			((PaRamp<<SX1272_PaRamp_PaRamp_gp)&SX1272_PaRamp_PaRamp_gm);
	Write( REG_COMMON_PARAMP,temp );
}
void SX1272Driver::OCP(bool ocpOn, uint8_t trim){
	uint8_t temp = (ocpOn<<SX1272_Ocp_OcpOn_bp) | (trim<<SX1272_Ocp_OcpTrim_gp);
	Write( REG_COMMON_OCP,temp );
}
void SX1272Driver::LNA(SX1272_LnaGain_t gain, SX1272_LnaBoost_t boostOn){
	uint8_t temp = gain | boostOn;
	Write( REG_COMMON_LNA,temp );
}
void SX1272Driver::DioMapping(uint8_t pin, SX1272_DioMapping_t map){
	uint8_t temp;
	switch(pin){
	case 0:
		temp = Read( REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio0Mapping_gm;
		temp |= map<<SX1272_Dio0Mapping_gp;
		Write( REG_COMMON_DIOMAPPING1,temp );
		break;
	case 1:
		temp = Read( REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio1Mapping_gm;
		temp |= map<<SX1272_Dio1Mapping_gp;
		Write( REG_COMMON_DIOMAPPING1,temp );
		break;
	case 2:
		temp = Read( REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio2Mapping_gm;
		temp |= map<<SX1272_Dio2Mapping_gp;
		Write( REG_COMMON_DIOMAPPING1,temp );
		break;
	case 3:
		temp = Read( REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio3Mapping_gm;
		temp |= map<<SX1272_Dio3Mapping_gp;
		Write( REG_COMMON_DIOMAPPING1,temp );
		break;
	case 4:
		temp = Read( REG_COMMON_DIOMAPPING2 );
		temp&= ~SX1272_Dio4Mapping_gm;
		temp |= map<<SX1272_Dio4Mapping_gp;
		Write( REG_COMMON_DIOMAPPING2,temp );
		break;
	case 5:
		temp = Read( REG_COMMON_DIOMAPPING2 );
		temp&= ~SX1272_Dio5Mapping_gm;
		temp |= map<<SX1272_Dio5Mapping_gp;
		Write( REG_COMMON_DIOMAPPING2,temp );
		break;
	default:
		break;
	}
}


void SX1272Driver::DioMapPreambleDetect(SX1272_DioMapPreambleDetect_t sel){
	uint8_t temp = Read( REG_COMMON_DIOMAPPING2 );
	temp&= ~LoRa_ModemStat_SignalDetected_bm;
	temp |= sel<<LoRa_ModemStat_SignalDetected_bp;
	Write( REG_COMMON_DIOMAPPING2,temp );
}


uint8_t SX1272Driver::Version(){
	return Read( REG_COMMON_VERSION );
}
void SX1272Driver::AgcRefLevel(uint8_t level){
	Write( REG_COMMON_AGCREF,level );
}
void SX1272Driver::AgcStepThreshold(SX1272_AgcStep_t step, uint8_t threshold){
	uint8_t temp;
	switch(step){
	case SX1272_AgcStep1:
		temp = Read( REG_COMMON_AGCTHRESH1 );
		temp &= ~SX1272_ArgStep1Thresl_gm;
		temp |= (threshold<<SX1272_ArgStep1Thresl_gp)&SX1272_ArgStep1Thresl_gm;
		Write( REG_COMMON_AGCTHRESH1, temp );
		break;
	case SX1272_AgcStep2:
		temp = Read( REG_COMMON_AGCTHRESH2 );
		temp &= ~SX1272_ArgStep1Thres2_gm;
		temp |= (threshold<<SX1272_ArgStep1Thres2_gp)&SX1272_ArgStep1Thres2_gm;
		Write( REG_COMMON_AGCTHRESH2, temp );
		break;
	case SX1272_AgcStep3:
		temp = Read( REG_COMMON_AGCTHRESH2 );
		temp &= ~SX1272_ArgStep1Thres3_gm;
		temp |= (threshold<<SX1272_ArgStep1Thres3_gp)&SX1272_ArgStep1Thres3_gm;
		Write( REG_COMMON_AGCTHRESH2, temp );
		break;
	case SX1272_AgcStep4:
		temp = Read( REG_COMMON_AGCTHRESH3 );
		temp &= ~SX1272_ArgStep1Thres4_gm;
		temp |= (threshold<<SX1272_ArgStep1Thres4_gp)&SX1272_ArgStep1Thres4_gm;
		Write( REG_COMMON_AGCTHRESH3, temp );
		break;
	case SX1272_AgcStep5:
		temp = Read( REG_COMMON_AGCTHRESH3 );
		temp &= ~SX1272_ArgStep1Thres5_gm;
		temp |= (threshold<<SX1272_ArgStep1Thres5_gp)&SX1272_ArgStep1Thres5_gm;
		Write( REG_COMMON_AGCTHRESH3, temp );
		break;
	}
}
void SX1272Driver::FastHopOn(bool sel){
	uint8_t temp = sel<<SX1272_PllHop_FastHopOn_bp;
	Write( REG_COMMON_PLLHOP, temp );
}
void SX1272Driver::Tcx0InputOn(bool sel){
	uint8_t temp = sel<<SX1272_Txco_FastHopOn_bp;
	Write( REG_COMMON_TCXO, temp );
}
void SX1272Driver::PaDac(SX1272_PaDac_t padac){
	Write( REG_COMMON_PADAC, padac );
}
void SX1272Driver::PllBandwidth(SX1272_PllBandwidth_t sel){
	uint8_t temp = (sel<<SX1272_PllBandwidth_gp);
	Write( REG_COMMON_PLL, temp&SX1272_PllBandwidth_gm );
}
void SX1272Driver::LowNoisePllBandwidth(SX1272_PllBandwidth_t sel){
	uint8_t temp = (sel<<SX1272_PllBandwidthLowPhaseNoise_gp);
	Write( REG_COMMON_PLLLOWPN, temp&SX1272_PllBandwidthLowPhaseNoise_gm );
}
uint8_t SX1272Driver::FormerTemp(){
	return Read( REG_COMMON_FORMERTEMP );
}
void SX1272Driver::BitRateFrac(uint8_t frac){
	Write( REG_COMMON_PLLLOWPN, frac&SX1272_BitrateFrac_gm  );
}

void SX1272Driver::FskModulationType(FSK_ModulationType_t sel){
	uint8_t temp =  Read(REG_COMMON_OPMODE);
	temp &= ~FSK_OpMode_ModulationType_gm;
	temp |= sel << FSK_OpMode_ModulationType_gp;
	Write( REG_COMMON_OPMODE, temp );
}
void SX1272Driver::FskModulationShaping_FSK(FSK_ModulationShaping_FSK_t sel){
	uint8_t temp =  Read(REG_COMMON_OPMODE);
	temp &= ~FSK_OpMode_ModulationShaping_gm;
	temp |= sel<<FSK_OpMode_ModulationShaping_gp;
	Write( REG_COMMON_OPMODE, temp );

}
void SX1272Driver::FskModulationShaping_OOK(FSK_ModulationShaping_OOK_t sel){
	uint8_t temp =  Read(REG_COMMON_OPMODE);
	temp &= ~FSK_OpMode_ModulationShaping_gm;
	temp |= sel<<FSK_OpMode_ModulationShaping_gp;
	Write( REG_COMMON_OPMODE, temp );
}
void SX1272Driver::FskMode(FSK_OpMode_t sel){
	uint8_t temp =  Read(REG_COMMON_OPMODE);
	temp &= ~FSK_OpMode_Mode_gm;
	temp |= sel<<FSK_OpMode_Mode_gp;
	Write( REG_COMMON_OPMODE, temp );
}

void SX1272Driver::FskBitrate(uint16_t bitrate){
	Write( REG_FSK_BITRATEMSB, (bitrate>>8) );
	Write( REG_FSK_BITRATELSB, (bitrate>>0) );
}
void SX1272Driver::FskFreqDev(uint16_t fdev){
	Write( REG_FSK_FDEVMSB, (fdev>>8) );
	Write( REG_FSK_FDEVLSB, (fdev>>0) );
}
void SX1272Driver::FskRestartRxOnCollision(bool sel){
	uint8_t temp =  Read(REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_RestartRxOnCollision_bm;
	temp |= sel<<FSK_RegRxConfig_RestartRxOnCollision_bp;
	Write( REG_FSK_RXCONFIG, temp );
}
void SX1272Driver::FskRestartRxWithougPllLock(){
	uint8_t temp =  Read(REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_RestartRxWithoutPllLock_bm;
	temp |= 1<<FSK_RegRxConfig_RestartRxWithoutPllLock_bp;
	Write( REG_FSK_RXCONFIG, temp );
}
void SX1272Driver::FskRestartRxWithPllLock(){
	uint8_t temp =  Read(REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_RestartRxWithPllLock_bm;
	temp |= 1<<FSK_RegRxConfig_RestartRxWithPllLock_bp;
	Write( REG_FSK_RXCONFIG, temp );
}
void SX1272Driver::FskAfcAutoOn(bool sel){
	uint8_t temp =  Read(REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_AfcAutoOn_bm;
	temp |= sel<<FSK_RegRxConfig_AfcAutoOn_bp;
	Write( REG_FSK_RXCONFIG, temp );
}
void SX1272Driver::FskAgcAutoOn(bool sel){
	uint8_t temp =  Read(REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_AgcAutoOn_bm;
	temp |= sel<<FSK_RegRxConfig_AgcAutoOn_bp;
	Write( REG_FSK_RXCONFIG, temp );
}
void SX1272Driver::FskRxTrigger(FSK_RxTrigger_t rxTrigger){
	uint8_t temp = Read(REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_RxTrigger_gm;
	temp |= (rxTrigger << FSK_RegRxConfig_RxTrigger_gp);
	Write( REG_FSK_RXCONFIG, temp );

}
void SX1272Driver::FskRssiConfig(
		uint8_t rssiOffset,
		FSK_RssiSmoothing_t rssiSmoothing,
		uint8_t rssiCollision,
		uint8_t rssiThres,
		uint8_t rssiVal){

	uint8_t temp = (rssiOffset << FSK_RegRssiConfig_RssiOffset_gp) | (rssiSmoothing << FSK_RegRssiConfig_RssiSmoothing_gp) ;
	Write( REG_FSK_RSSICONFIG, temp );
	Write( REG_FSK_RSSICOLLISION, rssiCollision );
	Write( REG_FSK_RSSITHRESH, rssiThres );
	Write( REG_FSK_RSSIVALUE, rssiVal );
}


void SX1272Driver::FskRxBw(FSK_RxBwMant_t RxBwMant, uint8_t RxBwExp){
	uint8_t temp = (RxBwMant << FSK_RegRxBw_RxBwMant_gp) | (RxBwExp << FSK_RegRxBw_RxBwExp_gp );
	Write( REG_FSK_RXBW, temp );
}

void SX1272Driver::FskAfcBw(uint8_t RxBwMantAfc, uint8_t RxBwExpAfc){
	uint8_t temp = (RxBwMantAfc << FSK_RegAfcBw_RxBwMantAfc_gp) | (RxBwExpAfc << FSK_RegAfcBw_RxBwExpAfc_gp );
	Write( REG_FSK_AFCBW, temp );
}

void SX1272Driver::FskOok(
		bool bitSyncOn,
		FSK_OokThresType_t OokThresType,
		FSK_OokPeakThresStep_t OokPeakTheshStep,
		uint8_t OokFixedThreshold,
		FSK_OokPeakThresDec_t OokPeakThreshDec,
		FSK_OokAverageOffset_t OokAverageOffset,
		FSK_OokAverageThreshFilt_t  OokAverageThreshFilt
		){

	uint8_t temp = (bitSyncOn << FSK_RegOokPeak_BitSyncOn_bp)
			| (OokThresType << FSK_RegOokPeak_OokThreshType_gp)
			| (OokPeakTheshStep << FSK_RegOokPeak_OokPeakTheshStep_gp) ;
	Write( REG_FSK_OOKPEAK, temp );
	Write( REG_FSK_OOKFIX, OokFixedThreshold );

	temp = (OokPeakThreshDec << FSK_RegOokAvg_OokPeakThreshDec_gp)
			| (OokAverageOffset<<FSK_RegOokAvg_OokAverageOffset_gp)
			| (OokAverageThreshFilt<<FSK_RegOokAvg_OokAverageThreshFilt_gp);
	Write( REG_FSK_OOKAVG, temp );

}


void SX1272Driver::FskAgcStart(){
	uint8_t temp =  Read(REG_FSK_AFCFEI);
	temp |= 1<<FSK_RegAfcFei_AgcStart_bp;
	Write( REG_FSK_AFCFEI, temp );
}
void SX1272Driver::FskAfcClear(){
	uint8_t temp =  Read(REG_FSK_AFCFEI);
	temp |= 1<<FSK_RegAfcFei_AfcClear_bp;
	Write( REG_FSK_AFCFEI, temp );
}
void SX1272Driver::FskAfcAutoClearOn(bool sel){
	uint8_t temp =  Read(REG_FSK_AFCFEI);
	temp &= ~FSK_RegAfcFei_AfcAutoClearOn_bm;
	temp |= 1<<FSK_RegAfcFei_AfcAutoClearOn_bp;
	Write( REG_FSK_AFCFEI, temp );
}
void SX1272Driver::FskAfcValue(uint16_t val){
	Write( REG_FSK_AFCMSB, (val>>8) );
	Write( REG_FSK_AFCLSB, (val>>0) );
}
void SX1272Driver::FskFeiValue(uint16_t val){
	Write( REG_FSK_FEIMSB, (val>>8)  );
	Write( REG_FSK_FEILSB, (val>>0)  );
}

void SX1272Driver::FskPreambleDetector(
		bool PreambleDetectorOn,
		FSK_PreambleDetectorSize_t PreambleDetectorSize,
		uint8_t PreambleDetectorTol
		){
	uint8_t temp = (PreambleDetectorOn<<FSK_RegPreambleDetect_PreambleDetectorOn_bp)
			| (PreambleDetectorSize <<FSK_RegPreambleDetect_PreambleDetectorSize_gp)
			| (PreambleDetectorTol<<FSK_RegPreambleDetect_PreambleDetectorTol_gp);
	Write( REG_FSK_PREAMBLEDETECT, temp );
}
void SX1272Driver::FskTimeout(uint8_t TimeoutRxRssi, uint8_t TimeoutRxPreamble, uint8_t TimeoutSignalSync, uint8_t InterPacketRxDelay){
	Write( REG_FSK_RXTIMEOUT1, TimeoutRxRssi );
	Write( REG_FSK_RXTIMEOUT2, TimeoutRxPreamble );
	Write( REG_FSK_RXTIMEOUT3, TimeoutSignalSync );
	Write( REG_FSK_RXDELAY, InterPacketRxDelay );
}


void SX1272Driver::FskRcCalStart(){
	uint8_t temp = Read(REG_FSK_OSC);
	temp |= FSK_RegOsc_RcCalStart_bm;
	Write( REG_FSK_OSC, temp );
}
void SX1272Driver::FskClkOut(FSK_ClkOut_t sel){
	Write( REG_FSK_OSC, sel );
}

void SX1272Driver::FskPreambleSize(uint16_t val){
	Write( REG_FSK_PREAMBLEMSB, (val>>8) );
	Write( REG_FSK_PREAMBLELSB, (val>>0) );
}

void SX1272Driver::FskSyncConfig(
		FSK_AutoRestartRxMode_t AutoRestartRxMode,
		FSK_PreamblePolarity_t PreamblePolarity,
		bool SyncOn,
		bool FifoFillCondition,
		uint8_t SyncSize
		){

	uint8_t temp = (AutoRestartRxMode << FSK_RegSyncConfig_AutoRestartRxMode_gp)
			| (PreamblePolarity<<FSK_RegSyncConfig_PreamblePolarity_bp)
			| (SyncOn<<FSK_RegSyncConfig_SyncOn_bp)
			| (FifoFillCondition << FSK_RegSyncConfig_FifoFillCondition_bp)
			| (SyncSize<<FSK_RegSyncConfig_SyncSize_gp);
	Write( REG_FSK_SYNCCONFIG, temp);
}


void SX1272Driver::FskSyncValue(uint64_t syncVal){
	Write( REG_FSK_SYNCVALUE1, (syncVal>>56));
	Write( REG_FSK_SYNCVALUE2, (syncVal>>48));
	Write( REG_FSK_SYNCVALUE3, (syncVal>>40));
	Write( REG_FSK_SYNCVALUE4, (syncVal>>32));
	Write( REG_FSK_SYNCVALUE5, (syncVal>>24));
	Write( REG_FSK_SYNCVALUE6, (syncVal>>16));
	Write( REG_FSK_SYNCVALUE7, (syncVal>>8));
	Write( REG_FSK_SYNCVALUE8, (syncVal>>0));
}
void SX1272Driver::FskPacketConfig(
		FSK_PacketFormat_t PacketFormat,
		FSK_DCFree_t DcFree,
		bool CrcOn,
		bool CrcAutoClearOff,
		FSK_AddressFiltering_t AddressFiltering,
		FSK_CrcWhitening_t CrcWhiteningType,
		FSK_DataMode_t DataMode,
		bool IoHomeOn,
		bool IoHomePowerFrame,
		bool BeaconOn
		){

	uint8_t config1 = (PacketFormat << FSK_RegPacketConfig1_PacketFormat_bp)
			| (DcFree<<FSK_RegPacketConfig1_DcFree_gp)
			| (CrcOn<<FSK_RegPacketConfig1_CrcOn_bp)
			| (CrcAutoClearOff << FSK_RegPacketConfig1_CrcAutoClearOff_bp)
			| (AddressFiltering<<FSK_RegPacketConfig1_AddressFiltering_gp)
			| (AddressFiltering<<FSK_RegPacketConfig1_CrcWhiteningType_bp);

	uint8_t config2  = (DataMode << FSK_RegPacketConfig2_DataMode_bp)
			| (IoHomeOn<<FSK_RegPacketConfig2_IoHomeOn_bp)
			| (IoHomePowerFrame<<FSK_RegPacketConfig2_IoHomePowerFrame_bp)
			| (BeaconOn<<FSK_RegPacketConfig2_BeaconOn_bp);

	Write( REG_FSK_PACKETCONFIG1, config1);
	Write( REG_FSK_PACKETCONFIG2, config2);

}


void SX1272Driver::FskPayloadLength(uint16_t val){
    uint8_t original_opmode = Read(REG_COMMON_OPMODE);
    if(settings.Modem == MODEM_FSK){
        FskMode(FSK_OpMode_Stdby);
    }
    else if(settings.Modem == MODEM_LORA){
        LoRaOpMode(LoRa_OpMode_STDBY_FskRegAccess);
    }
	uint8_t temp = Read(REG_FSK_PACKETCONFIG2);
	temp &= ~0x07;
	temp |= (val>>8);
	Write( REG_FSK_PACKETCONFIG2, temp);
	Write( REG_FSK_PAYLOADLENGTH, (val>>0));
    Write(REG_COMMON_OPMODE, original_opmode);
}
void SX1272Driver::FskNodeAddress(uint8_t val){
    uint8_t original_opmode = Read(REG_COMMON_OPMODE);
    if(settings.Modem == MODEM_FSK){
        FskMode(FSK_OpMode_Stdby);
    }
    else if(settings.Modem == MODEM_LORA){
        LoRaOpMode(LoRa_OpMode_STDBY_FskRegAccess);
    }
    Write(REG_FSK_NODEADRS,val);
    Write(REG_COMMON_OPMODE, original_opmode);
}
void SX1272Driver::FskBroadcastAddress(uint8_t val){
    uint8_t original_opmode = Read(REG_COMMON_OPMODE);
    if(settings.Modem == MODEM_FSK){
        FskMode(FSK_OpMode_Stdby);
    }
    else if(settings.Modem == MODEM_LORA){
        LoRaOpMode(LoRa_OpMode_STDBY_FskRegAccess);
    }
    Write(REG_FSK_BROADCASTADRS,val);
    Write(REG_COMMON_OPMODE, original_opmode);
}

void SX1272Driver::FskTxStartCondOnFifoThres(FSK_TxStartCondition_t startCond, uint8_t fifoThreshold ){
	uint8_t temp = (startCond << FSK_RegFifoThresh_TxStartCondition_bp) | (fifoThreshold << FSK_RegFifoThresh_FifoThreshold_gp );
	Write(REG_FSK_FIFOTHRESH, temp);
}

void SX1272Driver::FskSequenceStart(){
	uint8_t temp =  Read(REG_FSK_SEQCONFIG1);
	temp &= ~FSK_RegSeqConfig1_SequencerStart_bm;
	temp |= 1<<FSK_RegSeqConfig1_SequencerStart_bp;
	Write( REG_FSK_SEQCONFIG1, temp );
}
void SX1272Driver::FskSequenceStop(){
	uint8_t temp =  Read(REG_FSK_SEQCONFIG1);
	temp &= ~FSK_RegSeqConfig1_SequencerStop_bm;
	temp |= 1<<FSK_RegSeqConfig1_SequencerStop_bp;
	Write( REG_FSK_SEQCONFIG1, temp );
}

void SX1272Driver::FskSequencerConfig(
		FSK_IdleMode_t IdleMode,
		FSK_TransitionFromStart_t FromStart,
		FSK_LowPowerSelection_t LowPowerSelection,
		FSK_TransitionFromIdle_t FromIdle,
		FSK_TransitionFromTransmit_t FromTransmit,
		FSK_TransitionFromReceive_t FromReceive,
		FSK_TransitionFromRxTimeout_t FromRxTimeout,
		FSK_TransitionFromPacketReceived_t FromPacketReceived
		){
	uint8_t config1 = (IdleMode << FSK_RegSeqConfig1_IdleMode_bp)
			| (FromStart << FSK_RegSeqConfig1_FromStart_gp)
			| (LowPowerSelection << FSK_RegSeqConfig1_LowPowerSelection_bp)
			| (FromIdle << FSK_RegSeqConfig1_FromIdle_bp)
			| (FromTransmit << FSK_RegSeqConfig1_FromTransmit_bp);

	uint8_t config2 = (FromReceive << FSK_RegSeqConfig2_FromReceive_gp)
			| (FromRxTimeout << FSK_RegSeqConfig2_FromRxTimeout_gp)
			| (FromPacketReceived << FSK_RegSeqConfig2_FromPacketReceived_gp);

	Write(REG_FSK_SEQCONFIG1, config1);
	Write(REG_FSK_SEQCONFIG2, config2);
}


void SX1272Driver::FskTimer(
		FSK_TimerResolution_t Timer1,
		FSK_TimerResolution_t Timer2,
		uint8_t Timer1Coeff,
		uint8_t Timer2Coeff
		){

	uint8_t timerResol = (Timer1 << FSK_RegTimerResol_Timer1Resolution_gp) | (Timer2 << FSK_RegTimerResol_Timer2Resolution_gp);
	Write(REG_FSK_TIMERRESOL, timerResol);

	Write(REG_FSK_TIMER1COEF, Timer1Coeff);
	Write(REG_FSK_TIMER2COEF, Timer2Coeff);
}

void SX1272Driver::FskAutoImageCAlOn(bool sel){
    uint8_t temp = Read(REG_FSK_IMAGECAL);
    temp &= ~FSK_RegImageCal_AutoImageCalOn_bm;
    temp |= (sel << FSK_RegImageCal_AutoImageCalOn_bp);
    Write(REG_FSK_IMAGECAL, temp);
}
void SX1272Driver::FskImageCalStart(){
    uint8_t temp = Read(REG_FSK_IMAGECAL);
    temp |= (1 << FSK_RegImageCal_ImageCalStart_bp);
    Write(REG_FSK_IMAGECAL, temp);
}
bool SX1272Driver::FskImageCalRunning(){
	uint8_t temp = Read(REG_FSK_IMAGECAL);
	if( (temp & FSK_RegImageCal_ImageCalRunning_bm) == 0){
		return 0;
	}
	else{
		return 1;
	}
}
bool SX1272Driver::FskTempChange(){
	uint8_t temp = Read(REG_FSK_IMAGECAL);
	if( (temp & FSK_RegImageCal_TempChange_bm) == 0){
		return 0;
	}
	else{
		return 1;
	}
}
void SX1272Driver::FskTempThreshold(FSK_TempThreshold_t sel){
    uint8_t temp = Read(REG_FSK_IMAGECAL);
    temp &= ~FSK_RegImageCal_TempThreshold_gm;
    temp |= (sel << FSK_RegImageCal_TempThreshold_gp);
    Write(REG_FSK_IMAGECAL, temp);
}
void SX1272Driver::FskTempMonitorOff(bool sel){
    uint8_t temp = Read(REG_FSK_IMAGECAL);
    temp &= ~FSK_RegImageCal_TempMonitorOff_bm;
    temp |= (sel << FSK_RegImageCal_TempMonitorOff_bp);
    Write(REG_FSK_IMAGECAL, temp);
}
uint8_t SX1272Driver::FskTempValue(){
	return Read(REG_FSK_TEMP);
}

void SX1272Driver::FskLowBat(bool LowBatOn,FSK_LowBatTrim_t LowBatTrim){
	uint8_t temp = (LowBatOn<<FSK_RegLowBat_LowBatOn_bp) | (LowBatTrim << FSK_RegLowBat_LowBatTrim_gp);
	Write(REG_FSK_LOWBAT, temp);
}

bool SX1272Driver::FskIrqFlags(FSK_IrqFlags_t flag){
	if((flag >=0) && (flag<=7)){
		uint8_t temp = Read(REG_FSK_IRQFLAGS1);
		return (temp >> flag)&0x01;
	}
	else if((flag >=8) && (flag<=15)){
		uint8_t temp = Read(REG_FSK_IRQFLAGS2);
		return (temp >> (flag - 8))&0x01;
	}
}
void SX1272Driver::FskClearIrqFlag(FSK_IrqFlags_t flag){

	if( (flag ==   FSK_IrqFlags_SyncAddressMatch)
			||  (flag ==   FSK_IrqFlags_PreambleDetect)
			|| (flag ==   FSK_IrqFlags_Rssi)
			|| (flag ==   FSK_IrqFlags_LowBat)
			|| (flag ==   FSK_IrqFlags_FIfoOverrun)
			){
		if((flag >=0) && (flag<=7)){
			uint8_t temp = Read(REG_FSK_IRQFLAGS1);
			temp |= (1<<flag);
			Write(REG_FSK_IRQFLAGS1, temp);
		}
		else if((flag >=8) && (flag<=15)){
			uint8_t temp = Read(REG_FSK_IRQFLAGS2);
			temp |= (1<< (flag - 8));
			Write(REG_FSK_IRQFLAGS2, temp);
		}
	}
}

void SX1272Driver::LoRaOpMode(LoRa_OpMode_t mode){
    Write( REG_COMMON_OPMODE, mode);
}
void SX1272Driver::LoRaWriteFifoAddrPtr(uint8_t addr){
	Write( REG_LORA_FIFOADDRPTR, addr);
}
uint8_t SX1272Driver::LoRaReadFifoAddrPtr(){
	return Read( REG_LORA_FIFOADDRPTR);
}
void SX1272Driver::LoRaWriteFifoTxBaseAddr(uint8_t addr){
	Write( REG_LORA_FIFOTXBASEADDR, addr);
}
uint8_t SX1272Driver::LoRaReadFifoTxBaseAddr(){
	return Read( REG_LORA_FIFOTXBASEADDR);
}
void SX1272Driver::LoRaWriteFifoRxBaseAddr(uint8_t addr){
	Write( REG_LORA_FIFORXBASEADDR, addr);
}
uint8_t SX1272Driver::LoRaReadFifoRxBaseAddr(){
	return Read( REG_LORA_FIFORXBASEADDR);
}
uint8_t SX1272Driver::LoRaReadFifoRxCurrentAddr(){
	return Read( REG_LORA_FIFORXCURRENTADDR);
}
void SX1272Driver::LoRaIrqFlagsMask(uint8_t mask){
	Write( REG_LORA_IRQFLAGSMASK, mask);
}
bool SX1272Driver::LoRaIrqFlags(LoRa_IrqFlags_t flag){
	uint8_t temp = Read (REG_LORA_IRQFLAGS);
	if ((temp & flag)!= 0) return 1;
	else return 0;
}
void SX1272Driver::LoRaClearIrqFlags(LoRa_IrqFlags_t flag){
	Write( REG_LORA_IRQFLAGS, flag);
}

uint8_t SX1272Driver::LoRaRxPayloadBytes(){
	return Read (REG_LORA_RXNBBYTES);
}

uint16_t SX1272Driver::LoRaValidHeaderCount(){
	return (Read (REG_LORA_RXHEADERCNTVALUEMSB)<<8) |
			(Read (REG_LORA_RXHEADERCNTVALUELSB)<<0) ;
}
uint16_t SX1272Driver::LoRaValidPacketCount(){
	return (Read (REG_LORA_RXPACKETCNTVALUEMSB)<<8) |
			(Read (REG_LORA_RXPACKETCNTVALUELSB)<<0) ;
}
uint8_t SX1272Driver::LoRaModemStatus(LoRa_ModemStatus_t status){
	uint8_t temp =  Read (REG_LORA_MODEMSTAT);
	switch(status){
	case LoRa_ModemStatus_RxCodingRate:
		return (temp&LoRa_ModemStat_RxCodingRate_gm)>>LoRa_ModemStat_RxCodingRate_gp;
		break;
	case LoRa_ModemStatus_ModemClear:
		return (temp&LoRa_ModemStat_ModemClear_bm)>>LoRa_ModemStat_ModemClear_bp;
		break;
	case LoRa_ModemStatus_HeaderInfoValid:
		return (temp&LoRa_ModemStat_HeaderInfoValid_bm)>>LoRa_ModemStat_HeaderInfoValid_bp;
		break;
	case LoRa_ModemStatus_RxOnGoing:
		return (temp&LoRa_ModemStat_RxOnGoing_bm)>>LoRa_ModemStat_RxOnGoing_bp;
		break;
	case LoRa_ModemStatus_SignalSynchronized:
		return (temp&LoRa_ModemStat_SignalSynchronized_bm)>>LoRa_ModemStat_SignalSynchronized_bp;
		break;
	case LoRa_ModemStatus_SignalDetected:
		return (temp&LoRa_ModemStat_SignalDetected_bm)>>LoRa_ModemStat_SignalDetected_bp;
		break;
	default:
		return temp;
		break;
	}
}
uint8_t SX1272Driver::LoRaPacketSnr(){
	return Read (REG_LORA_PKTSNRVALUE);
}
uint8_t SX1272Driver::LoRaPacketRssi(){
	return Read (REG_LORA_PKTRSSIVALUE);
}
uint8_t SX1272Driver::LoRaRssi(){
	return Read (REG_LORA_RSSIVALUE);
}

bool SX1272Driver::LoRaPllTimeout(){
	uint8_t temp =  Read (REG_LORA_HOPCHANNEL);
	if( (temp & LoRa_HopChannel_PllTimeout_bm) != 0) return 1;
	else return 0;
}

bool SX1272Driver::LoRaCrcOnPayload(){
	uint8_t temp =  Read (REG_LORA_HOPCHANNEL);
	if( (temp & LoRa_HopChannel_CrcOnPayload_bm) != 0) return 1;
	else return 0;
}
uint8_t SX1272Driver::LoRaFhssPresentChannel(){
	uint8_t temp =  Read (REG_LORA_HOPCHANNEL);
	return temp & LoRa_HopChannel_FhssPresentChannel_gm;
}


void SX1272Driver::LoRaModemConfig(
		LoRa_ModemBw_t bandwidth,
		LoRa_ModemCodingRate_t codingRate,
		bool implicitHeaderModeOn,
		bool RxPayloadCrcOn,
		bool LowDataRateOptimize,
		LoRa_ModemSpreadingFactor_t spreadingfactor,
		bool txContinuousMode,
		bool ArcAutoOn
		){

	uint8_t config1 = bandwidth |
			codingRate |
			(implicitHeaderModeOn<<LoRa_ModemConfig1_ImplicitHeaderModeOn_bp) |
			(RxPayloadCrcOn<<LoRa_ModemConfig1_RxPayloadCrcOn_bp) |
			(LowDataRateOptimize<<LoRa_ModemConfig1_LowDataRateOptimize_bp);

	uint8_t config2 = spreadingfactor |
			(txContinuousMode<<LoRa_ModemConfig2_TxContMode_bp) |
			(ArcAutoOn<<LoRa_ModemConfig2_ArcAutoOn_bp);

	Write (REG_LORA_MODEMCONFIG1, config1);
	Write (REG_LORA_MODEMCONFIG2, config2);
}

void SX1272Driver::LoRaSymbTImeout(uint16_t timeout){
	uint8_t tempMSB = Read (REG_LORA_MODEMCONFIG2);
	tempMSB &= ~LoRa_ModemConfig2_SymbTimeoutMSB_gm;
	tempMSB |= (timeout>>8) & LoRa_ModemConfig2_SymbTimeoutMSB_gm ;
	Write (REG_LORA_MODEMCONFIG2, tempMSB);

	uint8_t tempLSB = timeout&0xFF;
	Write (REG_LORA_SYMBTIMEOUTLSB, tempLSB);

}
void SX1272Driver::LoRaPreambleLength(uint16_t len){
	Write (REG_LORA_PREAMBLEMSB, (len>>8)&0xFF);
	Write (REG_LORA_PREAMBLELSB, (len>>0)&0xFF);
}
void SX1272Driver::LoRaPayloadLength(uint8_t len){
	Write (REG_LORA_PAYLOADLENGTH, len);
}
void SX1272Driver::LoRaPayloadMaxLength(uint8_t len){
	Write (REG_LORA_PAYLOADMAXLENGTH, len);
}
void SX1272Driver::LoRaFreqHopPeriod(uint8_t period){
	Write (REG_LORA_HOPPERIOD, period);
}
uint8_t SX1272Driver::LoRaFifoRxByteAddrPtr(){
	return Read (REG_LORA_FIFORXBYTEADDR);
}

uint32_t SX1272Driver::LoRaFreqError(){
	uint8_t FEIMSB =  Read (REG_LORA_FEIMSB);
	uint8_t FEIMID =  Read (REG_LORA_FEIMID);
	uint8_t FEILSB =  Read (REG_LORA_FEILSB);

	return (FEIMSB<<16) | (FEIMID<<8)   |  (FEILSB);

}
uint8_t SX1272Driver::LoRaRssiWideband(){
	return Read (REG_LORA_RSSIWIDEBAND);
}
void SX1272Driver::LoRaDetectOptimize(LoRa_DetectOptimize_t sel){
	Write (REG_LORA_DETECTOPTIMIZE,0xC0 | sel);
}
void SX1272Driver::LoRaInvertIQSignal(bool sel){
	Write (REG_LORA_INVERTIQ,(sel<<LoRa_InvertIQ_bp));
}
void SX1272Driver::LoRaDetectThreshold(LoRa_DetectThreshold_t sel){
	Write (REG_LORA_DETECTIONTHRESHOLD,sel);
}
void SX1272Driver::LoRaSyncWord(uint8_t SyncWord){
	Write (REG_LORA_SYNCWORD,SyncWord);
}
