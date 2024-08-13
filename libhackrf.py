import ctypes as ct
import math

# HackRF error consts
HACKRF_SUCCESS = 0
HACKRF_TRUE = 1
HACKRF_TX_COMPLETE = -1
HACKRF_ERRORS = {
    -2: "HACKRF_ERROR_INVALID_PARAM",
    -5: "HACKRF_ERROR_NOT_FOUND",
    -6: "HACKRF_ERROR_BUSY",
    -11: "HACKRF_ERROR_NO_MEM",
    -1000: "HACKRF_ERROR_LIBUSB",
    -1001: "HACKRF_ERROR_THREAD",
    -1002: "HACKRF_ERROR_STREAMING_THREAD_ERR",
    -1003: "HACKRF_ERROR_STREAMING_STOPPED",
    -1004: "HACKRF_ERROR_STREAMING_EXIT_CALLED",
    -1005: "HACKRF_ERROR_USB_API_VERSION",
    -2000: "HACKRF_ERROR_NOT_LAST_DEVICE",
    -9999: "HACKRF_ERROR_OTHER",
}
UNKNOWN_ERROR = "UNKNOWN"

# load libhackrf
libhackrf = ct.CDLL('libhackrf.so.0')

# define data struct for hackrf device pointer as a C void pointer
p_hackrf_device = ct.c_void_p


class HackrfTransferStruct(ct.Structure):
    """
    Struct for hackrf_transfer
    """

    _fields_ = [("device", p_hackrf_device),
                ("buffer", ct.POINTER(ct.c_byte)),
                ("buffer_length", ct.c_int),
                ("valid_length", ct.c_int),
                ("rx_ctx", ct.c_void_p),
                ("tx_ctx", ct.c_void_p)]


class ReadSerialStruct(ct.Structure):
    """
    Struct for read_partid_serialno_t
    """

    _fields_ = [("part_id", ct.c_uint32 * 2),
                ("serial_no", ct.c_uint32 * 4)]


class HackrfDeviceListStruct(ct.Structure):
    """
    Struct for hackrf_device_list_t
    """

    _fields_ = [("serial_numbers", ct.POINTER(ct.c_char_p)),
                ("usb_board_ids", ct.c_void_p),
                ("usb_device_index", ct.POINTER(ct.c_int)),
                ("devicecount", ct.c_int),
                ("usb_devices", ct.POINTER(ct.c_void_p)),
                ("usb_devicecount", ct.c_int)]


# define callback
_callback = ct.CFUNCTYPE(ct.c_int, ct.POINTER(HackrfTransferStruct))

# extern ADDAPI int ADDCALL hackrf_init();
libhackrf.hackrf_init.restype = ct.c_int
libhackrf.hackrf_init.argtypes = []

# extern ADDAPI int ADDCALL hackrf_exit();
libhackrf.hackrf_exit.restype = ct.c_int
libhackrf.hackrf_exit.argtypes = []

# extern ADDAPI int ADDCALL hackrf_open(hackrf_device** device);
libhackrf.hackrf_open.restype = ct.c_int
libhackrf.hackrf_open.argtypes = [ct.POINTER(p_hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_open_by_serial(const char* const desired_serial_number, hackrf_device** device);
libhackrf.hackrf_open_by_serial.restype = ct.c_int
libhackrf.hackrf_open_by_serial.argtypes = [ct.POINTER(p_hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_close(hackrf_device* device);
libhackrf.hackrf_close.restype = ct.c_int
libhackrf.hackrf_close.argtypes = [p_hackrf_device]

# extern ADDAPI int ADDCALL hackrf_version_string_read(hackrf_device* device, char* version, uint8_t length);
libhackrf.hackrf_version_string_read.restype = ct.c_int
libhackrf.hackrf_version_string_read.argtypes = [p_hackrf_device, ct.POINTER(ct.c_char), ct.c_uint8]

# extern ADDAPI const char* ADDCALL hackrf_board_id_name(enum hackrf_board_id board_id);
libhackrf.hackrf_board_id_name.restype = ct.POINTER(ct.c_char)
libhackrf.hackrf_board_id_name.argtypes = []

# extern ADDAPI int ADDCALL hackrf_board_id_read(hackrf_device* device, uint8_t* value);
libhackrf.hackrf_board_id_read.restype = ct.c_int
libhackrf.hackrf_board_id_read.argtypes = [p_hackrf_device, ct.POINTER(ct.c_uint8)]

# extern ADDAPI int ADDCALL hackrf_board_partid_serialno_read(hackrf_device* device,
#                                                             read_partid_serialno_t* read_partid_serialno);
libhackrf.hackrf_board_partid_serialno_read.restype = ct.c_int
libhackrf.hackrf_board_partid_serialno_read.argtypes = [p_hackrf_device, ct.POINTER(ReadSerialStruct)]

# extern ADDAPI const char* ADDCALL hackrf_filter_path_name(const enum rf_path_filter path);
libhackrf.hackrf_filter_path_name.restype = ct.POINTER(ct.c_char)
libhackrf.hackrf_filter_path_name.argtypes = []

# extern ADDAPI hackrf_device_list_t* ADDCALL hackrf_device_list();
libhackrf.hackrf_device_list.restype = ct.POINTER(HackrfDeviceListStruct)
libhackrf.hackrf_device_list.argtypes = []

# extern ADDAPI int ADDCALL hackrf_device_list_open(hackrf_device_list_t *list, int idx, hackrf_device** device);
libhackrf.hackrf_device_list_open.restype = ct.c_int
libhackrf.hackrf_device_list_open.arg_types = [ct.POINTER(HackrfDeviceListStruct), ct.c_int, ct.POINTER(p_hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_set_sample_rate(hackrf_device* device, const double freq_hz);
libhackrf.hackrf_set_sample_rate.restype = ct.c_int
libhackrf.hackrf_set_sample_rate.argtypes = [p_hackrf_device, ct.c_double]

# extern ADDAPI int ADDCALL hackrf_set_freq(hackrf_device* device, const uint64_t freq_hz);
libhackrf.hackrf_set_freq.restype = ct.c_int
libhackrf.hackrf_set_freq.argtypes = [p_hackrf_device, ct.c_uint64]

# extern ADDAPI int ADDCALL hackrf_set_amp_enable(hackrf_device* device, const uint8_t value);
libhackrf.hackrf_set_amp_enable.restype = ct.c_int
libhackrf.hackrf_set_amp_enable.argtypes = [p_hackrf_device, ct.c_uint8]

# extern ADDAPI int ADDCALL hackrf_set_antenna_enable(hackrf_device* device, const uint8_t value);
libhackrf.hackrf_set_antenna_enable.restype = ct.c_int
libhackrf.hackrf_set_antenna_enable.argtypes = [p_hackrf_device, ct.c_uint8]

# extern ADDAPI int ADDCALL hackrf_set_lna_gain(hackrf_device* device, uint32_t value);
libhackrf.hackrf_set_lna_gain.restype = ct.c_int
libhackrf.hackrf_set_lna_gain.argtypes = [p_hackrf_device, ct.c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_vga_gain(hackrf_device* device, uint32_t value);
libhackrf.hackrf_set_vga_gain.restype = ct.c_int
libhackrf.hackrf_set_vga_gain.argtypes = [p_hackrf_device, ct.c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_txvga_gain(hackrf_device* device, uint32_t value);
libhackrf.hackrf_set_txvga_gain.restype = ct.c_int
libhackrf.hackrf_set_txvga_gain.argtypes = [p_hackrf_device, ct.c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_baseband_filter_bandwidth(hackrf_device* device, const uint32_t bandwidth_hz);
libhackrf.hackrf_set_baseband_filter_bandwidth.restype = ct.c_int
libhackrf.hackrf_set_baseband_filter_bandwidth.argtypes = [p_hackrf_device, ct.c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_freq_explicit(hackrf_device* device, const uint64_t if_freq_hz,
#                                                    const uint64_t lo_freq_hz, const enum rf_path_filter path);
libhackrf.hackrf_set_freq_explicit.restype = ct.c_int
libhackrf.hackrf_set_freq_explicit.argtypes = [ct.c_uint64, ct.c_uint64, ]

# extern ADDAPI int ADDCALL hackrf_set_sample_rate_manual(hackrf_device* device, const uint32_t freq_hz,
#                                                         const uint32_t divider);
libhackrf.hackrf_set_sample_rate_manual.restype = ct.c_int
libhackrf.hackrf_set_sample_rate_manual.argtypes = [p_hackrf_device, ct.c_uint32, ct.c_uint32]

# extern ADDAPI int ADDCALL hackrf_start_rx(hackrf_device* device, hackrf_sample_block_cb_fn callback, void* rx_ctx);
libhackrf.hackrf_start_rx.restype = ct.c_int
libhackrf.hackrf_start_rx.argtypes = [p_hackrf_device, _callback, ct.c_void_p]

# extern ADDAPI int ADDCALL hackrf_stop_rx(hackrf_device* device);
libhackrf.hackrf_stop_rx.restype = ct.c_int
libhackrf.hackrf_stop_rx.argtypes = [p_hackrf_device]

# extern ADDAPI int ADDCALL hackrf_start_tx(hackrf_device* device, hackrf_sample_block_cb_fn callback, void* tx_ctx);
libhackrf.hackrf_start_tx.restype = ct.c_int
libhackrf.hackrf_start_tx.argtypes = [p_hackrf_device, _callback, ct.c_void_p]

# extern ADDAPI int ADDCALL hackrf_stop_tx(hackrf_device* device);
libhackrf.hackrf_stop_tx.restype = ct.c_int
libhackrf.hackrf_stop_tx.argtypes = [p_hackrf_device]

# extern ADDAPI int ADDCALL hackrf_set_tx_block_complete_callback(hackrf_device* device,
#                                                                 hackrf_tx_block_complete_cb_fn callback);
libhackrf.hackrf_set_tx_block_complete_callback.restype = ct.c_int
libhackrf.hackrf_set_tx_block_complete_callback.argtypes = [p_hackrf_device, _callback]

# extern ADDAPI int ADDCALL hackrf_is_streaming(hackrf_device* device);
libhackrf.hackrf_is_streaming.restype = ct.c_int
libhackrf.hackrf_is_streaming.argtypes = [p_hackrf_device]

# extern ADDAPI int ADDCALL hackrf_max2837_read(hackrf_device* device, uint8_t register_number, uint16_t* value);
libhackrf.hackrf_max2837_read.restype = ct.c_int
libhackrf.hackrf_max2837_read.argtypes = [p_hackrf_device, ct.c_uint8, ct.POINTER(ct.c_uint16)]

# extern ADDAPI int ADDCALL hackrf_max2837_write(hackrf_device* device, uint8_t register_number, uint16_t value);
libhackrf.hackrf_max2837_write.restype = ct.c_int
libhackrf.hackrf_max2837_write.argtypes = [p_hackrf_device, ct.c_uint8, ct.c_uint16]

# extern ADDAPI int ADDCALL hackrf_si5351c_read(hackrf_device* device, uint16_t register_number, uint16_t* value);
libhackrf.hackrf_si5351c_read.restype = ct.c_int
libhackrf.hackrf_si5351c_read.argtypes = [p_hackrf_device, ct.c_uint16, ct.POINTER(ct.c_uint16)]

# extern ADDAPI int ADDCALL hackrf_si5351c_write(hackrf_device* device, uint16_t register_number, uint16_t value);
libhackrf.hackrf_si5351c_write.restype = ct.c_int
libhackrf.hackrf_si5351c_write.argtypes = [p_hackrf_device, ct.c_uint16, ct.c_uint16]

# extern ADDAPI int ADDCALL hackrf_rffc5071_read(hackrf_device* device, uint8_t register_number, uint16_t* value);
libhackrf.hackrf_rffc5071_read.restype = ct.c_int
libhackrf.hackrf_rffc5071_read.argtypes = [p_hackrf_device, ct.c_uint8, ct.POINTER(ct.c_uint16)]

# extern ADDAPI int ADDCALL hackrf_rffc5071_write(hackrf_device* device, uint8_t register_number, uint16_t value);
libhackrf.hackrf_rffc5071_write.restype = ct.c_int
libhackrf.hackrf_rffc5071_write.argtypes = [p_hackrf_device, ct.c_uint8, ct.c_uint16]

# extern ADDAPI int ADDCALL hackrf_spiflash_erase(hackrf_device* device);
libhackrf.hackrf_spiflash_erase.restype = ct.c_int
libhackrf.hackrf_spiflash_erase.argtypes = [p_hackrf_device]

# extern ADDAPI int ADDCALL hackrf_spiflash_read(hackrf_device* device, const uint32_t address,
#                                                const uint16_t length, unsigned char* data);
libhackrf.hackrf_spiflash_read.restype = ct.c_int
libhackrf.hackrf_spiflash_read.argtypes = [p_hackrf_device, ct.c_uint32, ct.c_uint16, ct.POINTER(ct.c_ubyte)]

# extern ADDAPI int ADDCALL hackrf_spiflash_write(hackrf_device* device, const uint32_t address,
#                                                 const uint16_t length, unsigned char* const data);
libhackrf.hackrf_spiflash_write.restype = ct.c_int
libhackrf.hackrf_spiflash_write.argtypes = [p_hackrf_device, ct.c_uint32, ct.c_uint16, ct.POINTER(ct.c_ubyte)]

# extern ADDAPI int ADDCALL hackrf_cpld_write(hackrf_device* device, unsigned char* const data,
#                                             const unsigned int total_length);
libhackrf.hackrf_cpld_write.restype = ct.c_int
libhackrf.hackrf_cpld_write.argtypes = [p_hackrf_device, ct.POINTER(ct.c_ubyte), ct.c_uint]

# extern ADDAPI const char* ADDCALL hackrf_error_name(enum hackrf_error errcode);
libhackrf.hackrf_error_name.restype = ct.POINTER(ct.c_char)
libhackrf.hackrf_error_name.argtypes = []

# extern ADDAPI uint32_t ADDCALL hackrf_get_transfer_queue_depth(hackrf_device* device);
libhackrf.hackrf_get_transfer_queue_depth.restype = ct.c_uint32
libhackrf.hackrf_get_transfer_queue_depth.argtypes = [p_hackrf_device]


class HackRF(object):

    # default values
    DEFAULT_INDEX = 0
    DEFAULT_BANDWIDTH = 5000000  # 5 MHz
    DEFAULT_LNA_GAIN = 32  # dB
    DEFAULT_VGA_GAIN = 8  # dB
    DEFAULT_TX_VGA_GAIN = 20  # dB
    DEFAULT_TCXO_REG = 0  # register for TCXO clock
    DEFAULT_SEND_WAIT = 1  # seconds

    HACKRF_BUFF_SIZE = 262144

    def __init__(self, device_index=DEFAULT_INDEX, mode='Rx'):
        """
        Wrapper for HackRF object
        :param device_index: HackRF index, default is 0
        :param mode: 'Rx' (default) or 'Tx'
        """

        # init lib
        result = libhackrf.hackrf_init()
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error initializing libhackrf: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        # device data
        self._dev_p = p_hackrf_device(None)
        self._buffer = bytearray()
        self._device_opened = False
        self._serial_num = ""
        self._tx_complete = False

        # RF data
        self._center_freq = 0
        self._sample_rate = 0
        self._bandwidth = self.DEFAULT_BANDWIDTH
        self._rx_lna_gain = self.DEFAULT_LNA_GAIN
        self._rx_vga_gain = self.DEFAULT_VGA_GAIN
        self._tx_vga_gain = self.DEFAULT_TX_VGA_GAIN

        # callback handles
        self._rx_cb_handle = None
        self._tx_cb_handle = None
        self._tx_complete_cb_handle = None

        # open device
        self._open(device_index)
        self._save_serial_num_string()

        if 'Tx' == mode:
            self.enable_amp()
            self.set_tx_vga_gain()

        else:
            self.disable_amp()
            self.set_rx_lna_gain()
            self.set_tx_vga_gain()

    def __del__(self):
        """
        Destructor. Closes HackRF object and exits libhackrf
        """

        self._close()

        result = libhackrf.hackrf_init()
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error exiting libhackrf: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

    def _open(self, device_index=DEFAULT_INDEX):
        """
        Open HackRF device
        :param device_index: HackRF index, default is 0
        """

        dev_list = libhackrf.hackrf_device_list()
        result = libhackrf.hackrf_device_list_open(dev_list, device_index, ct.pointer(self._dev_p))
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error opening HackRF device: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._device_opened = True

    def _close(self):
        """
        Close HackRF device
        """

        if not self._device_opened:
            return

        result = libhackrf.hackrf_close(self._dev_p)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error closing HackRF device: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._device_opened = False

    def _save_serial_num_string(self):
        """
        Save the HackRF device serial number as a string for quick access
        """

        serial_num = ReadSerialStruct()

        result = libhackrf.hackrf_board_partid_serialno_read(self._dev_p, serial_num)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error getting serial number: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        # convert the serial number from a C int array to a string
        for sni in serial_num.serial_no:
            self._serial_num += f"{sni:08x}"

    def get_serial_num(self):
        """
        Get the device serial number
        :return: Device serial number (string)
        """

        return self._serial_num

    def enable_amp(self):
        """
        Enable Rx/Tx amplifiers
        :return: Amplifier enabled (bool)
        """

        return libhackrf.hackrf_set_amp_enable(self._dev_p, 1)

    def disable_amp(self):
        """
        Disable Rx/Tx amplifiers
        :return: Amplifier enabled (bool)
        """

        return libhackrf.hackrf_set_amp_enable(self._dev_p, 0)

    def set_buffer(self, data):
        """
        Set the device buffer data
        :param data: Data bytes
        """

        # don't overwrite if data is already set
        if self._buffer:
            return

        if isinstance(data, bytes):
            self._buffer += data

        elif isinstance(data, bytearray):
            self._buffer += bytes(data)

        else:
            raise ValueError("Data must be a byte array!")

    def get_buffer(self):
        """
        Get the device buffer data
        :return: Buffer data (bytearray)
        """

        return self._buffer

    def clear_buffer(self):
        """
        Clear the device buffer data
        """

        self._buffer.clear()

    def set_freq(self, freq):
        """
        Set the device frequency for Rx/Tx
        :param freq: The frequency in Hz (int)
        """

        if not isinstance(freq, int):
            raise ValueError("Frequency must be an int!")

        result = libhackrf.hackrf_set_freq(self._dev_p, freq)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error setting frequency to {freq} Hz: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._center_freq = freq

    def get_freq(self):
        """
        Get the device frequency
        :return: Device frequency (int)
        """

        return self._center_freq

    def set_sample_rate(self, rate):
        """
        set the device sample rate
        :param rate: The sample rate in Hz (int)
        """

        if not isinstance(rate, int):
            raise ValueError("Sample rate must be an int!")

        result = libhackrf.hackrf_set_sample_rate(self._dev_p, rate)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error setting sample rate to {rate} Hz: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._sample_rate = rate

    def get_sample_rate(self):
        """
        Get the device sample rate
        :return: Device sample rate (int)
        """

        return self._sample_rate

    def set_bandwidth(self, bandwidth):
        """
        set the device sample bandwidth
        :param bandwidth: The bandwidth in Hz (int)
        """

        if not isinstance(bandwidth, int):
            raise ValueError("Bandwidth must be an int!")

        result = libhackrf.hackrf_set_baseband_filter_bandwidth(self._dev_p, bandwidth)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error setting bandwidth to {bandwidth} Hz: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._bandwidth = bandwidth

    def get_bandwidth(self):
        """
        Get the device bandwidth
        :return: Device bandwidth (int)
        """

        return self._bandwidth

    def set_rx_lna_gain(self, gain=DEFAULT_LNA_GAIN):
        """
        Set LNA gain for the device
        :param gain: The gain in dB (int)
        """

        if not isinstance(gain, int):
            raise ValueError("Gain must be an int!")

        # round DOWN to multiple of 8
        gain -= (gain % 8)

        result = libhackrf.hackrf_set_lna_gain(self._dev_p, gain)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error setting LNA gain to {gain} dB: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._rx_lna_gain = gain

    def get_rx_lna_gain(self):
        """
        Get the device LNA gain
        :return: LNA gain (int)
        """

        return self._rx_lna_gain

    def set_rx_vga_gain(self, gain=DEFAULT_VGA_GAIN):
        """
        Set VGA gain for the device
        :param gain: The gain in dB (int)
        """

        if not isinstance(gain, int):
            raise ValueError("Gain must be an int!")

        # round DOWN to multiple of 2
        gain -= (gain % 2)

        result = libhackrf.hackrf_set_vga_gain(self._dev_p, gain)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error setting VGA gain to {gain} dB: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._rx_vga_gain = gain

    def get_rx_vga_gain(self):
        """
        Get the device Rx VGA gain
        :return: Device Rx VGA (int)
        """

        return self._rx_vga_gain

    def set_tx_vga_gain(self, gain=DEFAULT_TX_VGA_GAIN):
        """
        Set TX VGA gain for the device
        :param gain: The gain in dB (int)
        """

        if not isinstance(gain, int):
            raise ValueError("Gain must be an int!")

        # round DOWN to multiple of 2
        gain -= (gain % 2)

        result = libhackrf.hackrf_set_txvga_gain(self._dev_p, gain)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Error setting VGA gain to {gain} dB: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        self._tx_vga_gain = gain

    def get_tx_vga_gain(self):
        """
        Get the device Tx VGA gain
        :return: Device Tx VGA (int)
        """

        return self._tx_vga_gain

    def start_rx(self):
        """
        Start Rx on device
        """

        def _rx_callback(hackrf_transfer):
            """
            Callback function that reads data from hackrf_transfer and copies it to buffer
            :param hackrf_transfer: HackrfTransferStruct
            """

            # copy data from hackrf_transfer to self.buffer
            buff_data = ct.cast(hackrf_transfer.contents.buffer,
                                ct.POINTER(ct.c_byte * hackrf_transfer.contents.buffer_length)).contents
            self._buffer += bytearray(buff_data)

            return HACKRF_SUCCESS

        # assign callback function to callback handle
        self._rx_cb_handle = _callback(_rx_callback)

        # start Rx callback thread
        result = libhackrf.hackrf_start_rx(self._dev_p, self._rx_cb_handle, None)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Failed to start Rx: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

    def stop_rx(self):
        """
        Stop Rx on device
        """

        if HACKRF_TRUE == libhackrf.hackrf_is_streaming(self._dev_p):

            result = libhackrf.hackrf_stop_rx(self._dev_p)
            if HACKRF_SUCCESS != result:
                raise IOError(f"Failed to stop Rx: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        # reset handle
        self._rx_cb_handle = None

        # clear buffer
        self._buffer.clear()

    def _set_tx_complete(self):
        """
        Set's actions when Tx block is done transferring
        """

        def _tx_callback_complete(hackrf_transfer):
            """
            Callback function that clears buffers after transmission is complete
            :param hackrf_transfer: HackrfTransferStruct
            """

            # clear buffers if transfer finished, data hasn't been cleared, and transmitted data matches buffer len
            if (self._tx_complete and self._buffer
                    and hackrf_transfer.contents.valid_length == 2 ** math.ceil(math.log2(len(self._buffer)))):

                # clear self buffer
                self._buffer.clear()

                # clear transfer buffer
                hackrf_transfer.contents.valid_length = self.HACKRF_BUFF_SIZE
                ct.memset(hackrf_transfer.contents.buffer, 0, hackrf_transfer.contents.buffer_length)

                # set flag
                self._tx_complete = False

            return HACKRF_SUCCESS

        # assign callback function to callback handle
        self._tx_complete_cb_handle = _callback(_tx_callback_complete)

        # set Tx complete callback
        result = libhackrf.hackrf_set_tx_block_complete_callback(self._dev_p, self._tx_complete_cb_handle)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Failed to set Tx complete callback: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

    def start_tx(self):
        """
        Start Tx on device
        """

        def _tx_callback(hackrf_transfer):
            """
            Callback function that writes data from self._buffer to hackrf_transfer
            :param hackrf_transfer: HackrfTransferStruct
            """

            # clear transfer buffer before writing to it
            ct.memset(hackrf_transfer.contents.buffer, 0, hackrf_transfer.contents.buffer_length)

            # check there is data to transmit
            data_len = len(self._buffer)

            # data is smaller than buffer
            if 0 < data_len <= self.HACKRF_BUFF_SIZE:

                # set length of data to transmit, as a power of 2, so that it matches hackrf len
                hackrf_transfer.contents.valid_length = 2 ** math.ceil(math.log2(data_len))

                # copy data from self._buffer to hackrf_transfer buffer
                buffer_array = (ct.c_char * data_len).from_buffer(self._buffer)
                ct.memmove(hackrf_transfer.contents.buffer, buffer_array, data_len)

                # set flag
                self._tx_complete = True

            # data is larger than buffer
            elif data_len > self.HACKRF_BUFF_SIZE:

                # get current block
                data_block = self._buffer[:self.HACKRF_BUFF_SIZE]
                hackrf_transfer.contents.valid_length = self.HACKRF_BUFF_SIZE

                # remove block from buffer
                self._buffer = self._buffer[self.HACKRF_BUFF_SIZE:]

                # copy data from block to hackrf_transfer buffer
                buffer_array = (ct.c_char * self.HACKRF_BUFF_SIZE).from_buffer(data_block)
                ct.memmove(hackrf_transfer.contents.buffer, buffer_array, self.HACKRF_BUFF_SIZE)

                # set flag
                self._tx_complete = False

            return HACKRF_SUCCESS

        # set tx_complete callback
        self._set_tx_complete()

        # assign callback function to callback handle
        self._tx_cb_handle = _callback(_tx_callback)

        # start Tx callback thread
        result = libhackrf.hackrf_start_tx(self._dev_p, self._tx_cb_handle, None)
        if HACKRF_SUCCESS != result:
            raise IOError(f"Failed to start Tx: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

    def stop_tx(self):
        """
        Stop Tx on device
        """

        if HACKRF_TRUE == libhackrf.hackrf_is_streaming(self._dev_p):

            result = libhackrf.hackrf_stop_tx(self._dev_p)
            if HACKRF_SUCCESS != result:
                raise IOError(f"Failed to stop Tx: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        # reset handles
        self._tx_cb_handle = None
        self._tx_complete_cb_handle = None
        self._tx_complete = True

        # clear buffer
        self._buffer.clear()

    def is_tcxo_connected(self):
        """
        Check if a TCXO clock is connected to the device
        """

        ret_val = ct.c_uint16(0)

        result = libhackrf.hackrf_si5351c_read(self._dev_p, ct.c_uint16(self.DEFAULT_TCXO_REG), ct.byref(ret_val))
        if HACKRF_SUCCESS != result:
            raise IOError(f"Failed to read TCXO register: {HACKRF_ERRORS.get(result, UNKNOWN_ERROR)}")

        return HACKRF_TRUE == ret_val.value
