#include "vl53l1.h"
#include "debug.h"

/*! vl53l1 platform functions */
int8_t vl53l1Init(VL53L1_Dev_t *dev) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	dev->comms_type = VL53L1_I2C;
	dev->comms_speed_khz = 400;

	status = VL53L1_DataInit(dev);
	if (status != VL53L1_ERROR_NONE) {
		return status;
	}

	status = VL53L1_StaticInit(dev);
	if (status != VL53L1_ERROR_NONE) {
		return status;
	}

	VL53L1_StopMeasurement(dev);
	/**
	 * DISTANCE MODE								Dark	Strong light
	 * VL53L1_DISTANCEMODE_SHORT		136		135
	 * VL53L1_DISTANCEMODE_MEDIUM		290		76
	 * VL53L1_DISTANCEMODE_LONG			360		73
	 */
	VL53L1_SetPresetMode(dev, VL53L1_PRESETMODE_LITE_RANGING);
	VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
	VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 10000);
	VL53L1_StartMeasurement(dev);

	return 0;
}

bool vl53l1Test(VL53L1_Dev_t *dev) {
	VL53L1_Error status;
	VL53L1_DeviceInfo_t info;
	status = VL53L1_GetDeviceInfo(dev, &info);
	return status == VL53L1_ERROR_NONE;
}



/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
    if (pdev->write(index, pdata, count, &pdev->i2c_slave_address)) {
        status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
        
	// if (i2cTofWrite(I2Cx, pdev->i2c_slave_address, index, count, pdata))
	// 	status = VL53L1_ERROR_CONTROL_INTERFACE;

	return status;
}

/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	if (pdev->read(index, pdata, count, &pdev->i2c_slave_address)) {
        status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    // if (i2cTofReadDma(I2Cx, pdev->i2c_slave_address, index, count, pdata))

	return status;
}

/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[1];
	buffer[0] = (uint8_t)(data);
	status = VL53L1_WriteMulti(pdev, index, buffer, 1);
	return status;
}

/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t data) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[2];
	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data & 0x00FF);
	status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_WORD);
	return status;
}

/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 *
 */

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[1];
	status = VL53L1_ReadMulti(pdev, index, buffer, 1);
	*pdata = buffer[0];
	return status;
}

/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t *pdata) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t  buffer[2];
	status = VL53L1_ReadMulti(pdev, index, buffer, VL53L1_BYTES_PER_WORD);
	*pdata = (uint16_t)(((uint16_t)(buffer[0]) << 8) + (uint16_t)buffer[1]);

	return status;
}

/**
 * @brief  Implements a programmable wait in us
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_us   : integer wait in micro seconds
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us) {
	uint32_t wait_ms = (wait_us + 500) / 1000;
	if (wait_ms == 0)
		pdev->delay(1);
	else
		pdev->delay(wait_ms);
	return VL53L1_ERROR_UNDEFINED;
}

/**
 * @brief  Implements a programmable wait in ms
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_ms   : integer wait in milliseconds
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms) {
	pdev->delay(wait_ms);
	return VL53L1_ERROR_UNDEFINED;
}

/**
 * @brief Gets current system tick count in [ms]
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   time_ms : current time in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_GetTickCount(VL53L1_Dev_t *pdev, uint32_t *ptime_ms) {
	*ptime_ms = pdev->millis();
	return VL53L1_ERROR_UNDEFINED;
}

/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms, uint16_t index, 
																		uint8_t value, uint8_t mask, uint32_t poll_delay_ms) {
	/**
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint32_t start_time_ms = 0;
	uint32_t current_time_ms = 0;
	uint8_t byte_value = 0;
	uint8_t found = 0;
#ifdef VL53L1_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	_LOG_STRING_BUFFER(register_name);

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53L1_LOG_ENABLE
	/* look up register name */
	VL53L1_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	VL53L1_GetTickCount(pdev, &start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53L1_TRACE_FUNCTION_NONE);

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0)) {
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		/*if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);
		*/

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(pdev, &current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	/* Restore function logging */
	_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}