#define MAX_WRITE_THREAD_BUFFER_SIZE 8


#ifndef _BOOLEAN_T_DEFINED_
  #define _BOOLEAN_T_DEFINED_

  typedef signed char        boolean_t;

  #if !defined(TRUE)
    #define TRUE               (boolean_t)0x01
  #endif

  #if !defined(FALSE)
    #define FALSE              (boolean_t)0x00
  #endif

  #define BOOL_NOT(b) (b^TRUE)

#endif /* _BOOLEAN_T_DEFINED_ */

typedef enum {
  MIN_RF_DEV_TYPE = 0,
  /*!\brief device is ExpressMIMO */
  EXMIMO_DEV,
  /*!\brief device is USRP B200/B210*/
  USRP_B200_DEV,
  /*!\brief device is USRP X300/X310*/
  USRP_X300_DEV,
  /*!\brief device is USRP N300/N310*/
  USRP_N300_DEV,
  /*!\brief device is BLADE RF*/
  BLADERF_DEV,
  /*!\brief device is LMSSDR (SoDeRa)*/
  LMSSDR_DEV,
  /*!\brief device is Iris */
  IRIS_DEV,
  /*!\brief device is NONE*/
  NONE_DEV,
  /*!\brief device is ADRV9371_ZC706 */
  ADRV9371_ZC706_DEV,
  /*!\brief device is UEDv2 */
  UEDv2_DEV,
  RFSIMULATOR,
  MAX_RF_DEV_TYPE
} dev_type_t;

typedef long openair0_timestamp;

/*!\brief structure holds the parameters to configure USRP devices*/
typedef struct openair0_device_t openair0_device;

/*!\brief structure holds the parameters to configure USRP devices */
struct openair0_device_t {
    /*!tx write thread*/
    openair0_thread_t write_thread;

    /*!brief Module ID of this device */
    int Mod_id;

    /*!brief Component Carrier ID of this device */
    int CC_id;

    /*!brief Type of this device */
    dev_type_t type;

    /*!brief Transport protocol type that the device supports (in case I/Q samples need to be transported) */
    transport_type_t transp_type;

    /*!brief Type of the device's host (RAU/RRU) */
    host_type_t host_type;

    /* !brief RF frontend parameters set by application */
    openair0_config_t *openair0_cfg;

    /* !brief ETH params set by application */
    eth_params_t *eth_params;
    //! record player data, definition in record_player.h
    recplay_state_t *recplay_state;
    /* !brief Indicates if device already initialized */
    int is_init;


    /*!brief Can be used by driver to hold internal structure*/
    void *priv;

    /* Functions API, which are called by the application*/

    /*! \brief Called to start the transceiver. Return 0 if OK, < 0 if error
        @param device pointer to the device structure specific to the RF hardware target
    */
    int (*trx_start_func)(openair0_device *device);

    /*! \brief Called to configure the device
         @param device pointer to the device structure specific to the RF hardware target
     */


    int (*trx_config_func)(openair0_device *device, openair0_config_t *openair0_cfg);

    /*! \brief Called to send a request message between RAU-RRU on control port
        @param device pointer to the device structure specific to the RF hardware target
        @param msg pointer to the message structure passed between RAU-RRU
        @param msg_len length of the message
    */
    int (*trx_ctlsend_func)(openair0_device *device, void *msg, ssize_t msg_len);

    /*! \brief Called to receive a reply  message between RAU-RRU on control port
        @param device pointer to the device structure specific to the RF hardware target
        @param msg pointer to the message structure passed between RAU-RRU
        @param msg_len length of the message
    */
    int (*trx_ctlrecv_func)(openair0_device *device, void *msg, ssize_t msg_len);

    /*! \brief Called to send samples to the RF target
        @param device pointer to the device structure specific to the RF hardware target
        @param timestamp The timestamp at whicch the first sample MUST be sent
        @param buff Buffer which holds the samples (2 dimensional)
        @param nsamps number of samples to be sent
        @param number of antennas
        @param flags flags must be set to TRUE if timestamp parameter needs to be applied
    */
    int
    (*trx_write_func)(openair0_device *device, openair0_timestamp timestamp, void **buff, int nsamps, int antenna_id,
                      int flags);

    /*! \brief Called to send samples to the RF target
        @param device pointer to the device structure specific to the RF hardware target
        @param timestamp The timestamp at whicch the first sample MUST be sent
        @param buff Buffer which holds the samples (1 dimensional)
        @param nsamps number of samples to be sent
        @param antenna_id index of the antenna if the device has multiple anteannas
        @param flags flags must be set to TRUE if timestamp parameter needs to be applied
    */
    int
    (*trx_write_func2)(openair0_device *device, openair0_timestamp timestamp, void *buff, int nsamps, int antenna_id,
                       int flags);

    /*! \brief Receive samples from hardware.
     * Read \ref nsamps samples from each channel to buffers. buff[0] is the array for
     * the first channel. *ptimestamp is the time at which the first sample
     * was received.
     * \param device the hardware to use
     * \param[out] ptimestamp the time at which the first sample was received.
     * \param[out] buff An array of pointers to buffers for received samples. The buffers must be large enough to hold the number of samples \ref nsamps.
     * \param nsamps Number of samples. One sample is 2 byte I + 2 byte Q => 4 byte.
     * \param num_antennas number of antennas from which to receive samples
     * \returns the number of sample read
     */

    int (*trx_read_func)(openair0_device *device, openair0_timestamp *ptimestamp, void **buff, int nsamps,
                         int num_antennas);

    /*! \brief Receive samples from hardware, this version provides a single antenna at a time and returns.
     * Read \ref nsamps samples from each channel to buffers. buff[0] is the array for
     * the first channel. *ptimestamp is the time at which the first sample
     * was received.
     * \param device the hardware to use
     * \param[out] ptimestamp the time at which the first sample was received.
     * \param[out] buff A pointers to a buffer for received samples. The buffer must be large enough to hold the number of samples \ref nsamps.
     * \param nsamps Number of samples. One sample is 2 byte I + 2 byte Q => 4 byte.
     * \param antenna_id Index of antenna from which samples were received
     * \returns the number of sample read
     */
    int
    (*trx_read_func2)(openair0_device *device, openair0_timestamp *ptimestamp, void *buff, int nsamps, int *antenna_id);

    /*! \brief print the device statistics
     * \param device the hardware to use
     * \returns  0 on success
     */
    int (*trx_get_stats_func)(openair0_device *device);

    /*! \brief Reset device statistics
     * \param device the hardware to use
     * \returns 0 in success
     */
    int (*trx_reset_stats_func)(openair0_device *device);

    /*! \brief Terminate operation of the transceiver -- free all associated resources
     * \param device the hardware to use
     */
    void (*trx_end_func)(openair0_device *device);

    /*! \brief Stop operation of the transceiver
     */
    int (*trx_stop_func)(openair0_device *device);

    /* Functions API related to UE*/

    /*! \brief Set RX feaquencies
     * \param device the hardware to use
     * \param openair0_cfg RF frontend parameters set by application
     * \param exmimo_dump_config  dump EXMIMO configuration
     * \returns 0 in success
     */
    int (*trx_set_freq_func)(openair0_device *device, openair0_config_t *openair0_cfg, int exmimo_dump_config);

    /*! \brief Set gains
     * \param device the hardware to use
     * \param openair0_cfg RF frontend parameters set by application
     * \returns 0 in success
     */
    int (*trx_set_gains_func)(openair0_device *device, openair0_config_t *openair0_cfg);

    /*! \brief RRU Configuration callback
     * \param idx RU index
     * \param arg pointer to capabilities or configuration
     */
    void (*configure_rru)(int idx, void *arg);

/*! \brief Pointer to generic RRU private information
   */

    void *thirdparty_priv;

    /*! \brief Callback for Third-party RRU Initialization routine
       \param device the hardware configuration to use
     */
    int (*thirdparty_init)(openair0_device *device);

    /*! \brief Callback for Third-party RRU Cleanup routine
       \param device the hardware configuration to use
     */
    int (*thirdparty_cleanup)(openair0_device *device);

    /*! \brief Callback for Third-party start streaming routine
       \param device the hardware configuration to use
     */
    int (*thirdparty_startstreaming)(openair0_device *device);

    /*! \brief RRU Configuration callback
     * \param idx RU index
     * \param arg pointer to capabilities or configuration
     */
    int (*trx_write_init)(openair0_device *device);

    /* \brief Get internal parameter
     * \param id parameter to get
     * \return a pointer to the parameter
     */
    void *(*get_internal_parameter)(char *id);
};

/*! \brief Clock source types */
typedef enum {
  //! this means the paramter has not been set
  unset=-1,
  //! This tells the underlying hardware to use the internal reference
  internal=0,
  //! This tells the underlying hardware to use the external reference
  external=1,
  //! This tells the underlying hardware to use the gpsdo reference
  gpsdo=2
} clock_source_t;

/*! \brief RF frontend parameters set by application */
typedef struct {
  //! Module ID for this configuration
  int Mod_id;
  //! device log level
  int log_level;
  //! duplexing mode
  duplex_mode_t duplex_mode;
  //! number of downlink resource blocks
  int num_rb_dl;
  //! number of samples per frame
  unsigned int  samples_per_frame;
  //! the sample rate for both transmit and receive.
  double sample_rate;
  //! flag to indicate that the device is doing mmapped DMA transfers
  int mmapped_dma;
  //! offset in samples between TX and RX paths
  int tx_sample_advance;
  //! samples per packet on the fronthaul interface
  int samples_per_packet;
  //! number of RX channels (=RX antennas)
  int rx_num_channels;
  //! number of TX channels (=TX antennas)
  int tx_num_channels;
  //! \brief RX base addresses for mmapped_dma
  int32_t *rxbase[4];
  //! \brief TX base addresses for mmapped_dma
  int32_t *txbase[4];
  //! \brief Center frequency in Hz for RX.
  //! index: [0..rx_num_channels[
  double rx_freq[4];
  //! \brief Center frequency in Hz for TX.
  //! index: [0..rx_num_channels[ !!! see lte-ue.c:427 FIXME iterates over rx_num_channels
  double tx_freq[4];
  //! \brief memory
  //! \brief Pointer to Calibration table for RX gains
  rx_gain_calib_table_t *rx_gain_calib_table;
  //! mode for rxgain (ExpressMIMO2)
  rx_gain_t rxg_mode[4];
  //! \brief Gain for RX in dB.
  //! index: [0..rx_num_channels]
  double rx_gain[4];
  //! \brief Gain offset (for calibration) in dB
  //! index: [0..rx_num_channels]
  double rx_gain_offset[4];
  //! gain for TX in dB
  double tx_gain[4];
  //! RX bandwidth in Hz
  double rx_bw;
  //! TX bandwidth in Hz
  double tx_bw;
  //! clock source
  clock_source_t clock_source;
  //! timing_source
  clock_source_t time_source;
  //! Manual SDR IP address
  //#if defined(EXMIMO) || defined(OAI_USRP) || defined(OAI_BLADERF) || defined(OAI_LMSSDR)
  char *sdr_addrs;
  //! Auto calibration flag
  int autocal[4];
  //! rf devices work with x bits iqs when oai have its own iq format
  //! the two following parameters are used to convert iqs
  int iq_txshift;
  int iq_rxrescale;
  //! Configuration file for LMS7002M
  char *configFilename;
  //! remote IP/MAC addr for Ethernet interface
  char *remote_addr;
  //! remote port number for Ethernet interface
  unsigned int remote_port;
  //! local IP/MAC addr for Ethernet interface (eNB/BBU, UE)
  char *my_addr;
  //! local port number for Ethernet interface (eNB/BBU, UE)
  unsigned int my_port;
  //! record player configuration, definition in record_player.h
  uint32_t       recplay_mode;
  recplay_conf_t *recplay_conf;
  //! number of samples per tti
  unsigned int  samples_per_tti;
  //! the sample rate for receive.
  double rx_sample_rate;
  //! the sample rate for transmit.
  double tx_sample_rate;
  //! check for threequarter sampling rate
  int8_t threequarter_fs;
} openair0_config_t;