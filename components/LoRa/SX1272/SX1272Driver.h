/*
 * SX1272Driver.h
 *
 *  Created on: Mar 28, 2017
 *      Author: emon1
 */

#ifndef COMPONENTS_LORA_SX1272_SX1272DRIVER_H_
#define COMPONENTS_LORA_SX1272_SX1272DRIVER_H_

#include "../HAL/LoRaHAL.h"
#include "SX1272_Reg.h"


class SX1272Driver: public LoRaHAL {
public:

	typedef enum
	{
		MODEM_FSK = 0,
		MODEM_LORA,
	}RadioModems_t;

	typedef struct{
		RadioModems_t Modem;
	}Settings_t;

	Settings_t settings;


	SX1272Driver();
	virtual ~SX1272Driver();

	void SetModem(RadioModems_t modem);
	void WriteFifo( uint8_t *buffer, uint8_t size );
	void ReadFifo( uint8_t *buffer, uint8_t size) ;

	void LongRangeMode(SX1272_LongRangeMode_t mode);
	void Frf(uint32_t freq);
	void PaConfig(SX1272_PaSelect_t PAsel, uint8_t outputPower);
	void PaRamp(bool LowPnTxPllOff , SX1272_PaRamp_t PaRamp);
	void OCP(bool ocpOn, uint8_t trim);
	void LNA(SX1272_LnaGain_t gain, SX1272_LnaBoost_t boostOn);
	void DioMapping(uint8_t pin, SX1272_DioMapping_t map);
	void DioMapPreambleDetect(SX1272_DioMapPreambleDetect_t sel);

	uint8_t Version();
	void AgcRefLevel(uint8_t level);
	void AgcStepThreshold(SX1272_AgcStep_t step, uint8_t threshold);
	void FastHopOn(bool sel);
	void Tcx0InputOn(bool sel);
	void PaDac(SX1272_PaDac_t sel);
	void PllBandwidth(SX1272_PllBandwidth_t sel);
	void LowNoisePllBandwidth(SX1272_PllBandwidth_t sel);
	uint8_t FormerTemp();
	void BitRateFrac(uint8_t frac);

	void FskModulationType(FSK_ModulationType_t sel);
	void FskModulationShaping_FSK(FSK_ModulationShaping_FSK_t sel);
	void FskModulationShaping_OOK(FSK_ModulationShaping_OOK_t sel);
	void FskMode(FSK_OpMode_t sel);

	void FskBitrate(uint16_t bitrate);
	void FskFreqDev(uint16_t fdev);
	void FskRestartRxOnCollision(bool sel);
	void FskRestartRxWithougPllLock();
	void FskRestartRxWithPllLock();
	void FskAfcAutoOn(bool sel);
	void FskAgcAutoOn(bool sel);
	void FskRxTrigger(FSK_RxTrigger_t sel);//needs table to be checked (Table 23 for description on datasheet)
	void FskRssiConfig(
			uint8_t rssiOffset,
			FSK_RssiSmoothing_t rssiSmoothing,
			uint8_t rssiCollision,
			uint8_t rssiThres,
			uint8_t rssiVal);
	void FskRxBw(FSK_RxBwMant_t RxBwMant, uint8_t RxBwExp);
	void FskAfcBw(uint8_t RxBwMantAfc, uint8_t RxBwExpAfc);
	void FskOok(
			bool bitSyncOn,
			FSK_OokThresType_t OokThresType,
			FSK_OokPeakThresStep_t OokPeakTheshStep,
			uint8_t OokFixedThreshold,
			FSK_OokPeakThresDec_t OokPeakThreshDec,
			FSK_OokAverageOffset_t OokAverageOffset,
			FSK_OokAverageThreshFilt_t  OokAverageThreshFilt
			);
	void FskAgcStart();
	void FskAfcClear();
	void FskAfcAutoClearOn(bool sel);
	void FskAfcValue(uint16_t val);
	void FskFeiValue(uint16_t val);
	void FskPreambleDetector(
			bool PreambleDetectorOn,
			FSK_PreambleDetectorSize_t PreambleDetectorSize,
			uint8_t PreambleDetectorTol
			);
	void FskTimeout(uint8_t TimeoutRxRssi, uint8_t TimeoutRxPreamble, uint8_t TimeoutSignalSync, uint8_t InterPacketRxDelay);

	void FskRcCalStart();
	void FskClkOut(FSK_ClkOut_t sel);
	void FskPreambleSize(uint16_t val);

	void FskSyncConfig(
			FSK_AutoRestartRxMode_t AutoRestartRxMode,
			FSK_PreamblePolarity_t PreamblePolarity,
			bool SyncOn,
			bool FifoFillCondition,
			uint8_t SyncSize
			);


	void FskSyncValue(uint64_t val);

	void FskPacketConfig(
			FSK_PacketFormat_t PacketFormat,
			FSK_DCFree_t DcFree,
			bool CrcOn,
			bool CrcAutoClearOff,
			FSK_AddressFiltering_t AddressFiltering,
			FSK_CrcWhitening_t CrcWhiteningType
	);
	void FskPacketConfig(
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
			);

	void FskDataMode(FSK_DataMode_t sel);
	void FskIoHomeOn(bool sel);
	void FskIoHomePowerFrame(bool sel);
	void FskBeaconOn(bool sel);



	void FskPayloadLength(uint16_t val);
	void FskNodeAddress(uint8_t val);
	void FskBroadcastAddress(uint8_t val);
	void FskTxStartCondOnFifoThres(FSK_TxStartCondition_t startCond, uint8_t fifoThreshold );
	void FskSequenceStart();
	void FskSequenceStop();
	void FskSequencerConfig(
			FSK_IdleMode_t IdleMode,
			FSK_TransitionFromStart_t FromStart,
			FSK_LowPowerSelection_t LowPowerSelection,
			FSK_TransitionFromIdle_t FromIdle,
			FSK_TransitionFromTransmit_t FromTransmit,
			FSK_TransitionFromReceive_t FromReceive,
			FSK_TransitionFromRxTimeout_t FromRxTimeout,
			FSK_TransitionFromPacketReceived_t FromPacketReceived
			);
	void FskTimer(
			FSK_TimerResolution_t Timer1,
			FSK_TimerResolution_t Timer2,
			uint8_t Timer1Coeff,
			uint8_t Timer2Coeff
			);
	void FskTimerResolution(FSK_TimerResolution_t Timer1, FSK_TimerResolution_t Timer2);
	void FskTimerCoefficient(uint8_t Timer1Coeff, uint8_t Timer2Coeff);

	void FskAutoImageCAlOn(bool sel);
	void FskImageCalStart();
	bool FskImageCalRunning();
	bool FskTempChange();
	void FskTempThreshold(FSK_TempThreshold_t sel);
	void FskTempMonitorOff(bool sel);
	uint8_t FskTempValue();

	void FskLowBat(bool LowBatOn,FSK_LowBatTrim_t LowBatTrim);

	bool FskIrqFlags(FSK_IrqFlags_t flag);
	void FskClearIrqFlag(FSK_IrqFlags_t flag);


	void LoRaOpMode(LoRa_OpMode_t mode);
	void LoRaWriteFifoAddrPtr(uint8_t addr);
	uint8_t LoRaReadFifoAddrPtr();
	void LoRaWriteFifoTxBaseAddr(uint8_t addr);
	uint8_t LoRaReadFifoTxBaseAddr();
	void LoRaWriteFifoRxBaseAddr(uint8_t addr);
	uint8_t LoRaReadFifoRxBaseAddr();
	uint8_t LoRaReadFifoRxCurrentAddr();
	void LoRaIrqFlagsMask(uint8_t mask);
	bool LoRaIrqFlags(LoRa_IrqFlags_t flag);
	void LoRaClearIrqFlags(LoRa_IrqFlags_t flag);
	uint8_t LoRaRxPayloadBytes();
	uint16_t LoRaValidHeaderCount();
	uint16_t LoRaValidPacketCount();
	uint8_t LoRaModemStatus(LoRa_ModemStatus_t status);
	uint8_t LoRaPacketSnr();
	uint8_t LoRaPacketRssi();
	uint8_t LoRaRssi();
	bool LoRaPllTimeout();
	bool LoRaCrcOnPayload();
	uint8_t LoRaFhssPresentChannel();
	void LoRaModemConfig(
					LoRa_ModemBw_t bandwidth,
					LoRa_ModemCodingRate_t codingRate,
					bool implicitHeaderModeOn,
					bool RxPayloadCrcOn,
					bool LowDataRateOptimize,
					LoRa_ModemSpreadingFactor_t spreadingfactor,
					bool txContinuousMode,
					bool ArcAutoOn
					);
	void LoRaSymbTImeout(uint16_t timeout);
	void LoRaPreambleLength(uint16_t len);
	void LoRaPayloadLength(uint8_t len);
	void LoRaPayloadMaxLength(uint8_t len);
	void LoRaFreqHopPeriod(uint8_t period);
	uint8_t LoRaFifoRxByteAddrPtr();
	uint32_t LoRaFreqError();
	uint8_t LoRaRssiWideband();
	void LoRaDetectOptimize(LoRa_DetectOptimize_t sel);
	void LoRaInvertIQSignal(bool sel);
	void LoRaDetectThreshold(LoRa_DetectThreshold_t sel);
	void LoRaSyncWord(uint8_t SyncWord);


};

#endif /* COMPONENTS_LORA_SX1272_SX1272DRIVER_H_ */
