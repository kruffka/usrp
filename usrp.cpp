#include <uhd/version.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/version.hpp>
#include "common_lib.h"
#include <xmmintrin.h>


// uhd safe main
#include <uhd/utils/safe_main.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <csignal>

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}
// end of uhd safe main


// #define __AVX2__

#ifdef __SSE4_1__
  #include <smmintrin.h>
#endif

#ifdef __AVX2__
  #include <immintrin.h>
#endif

#ifdef __arm__
  #include <arm_neon.h>
#endif

#define LOG_I(A,B...) printf(B)
#define LOG_E(A,B...) printf(B)
#define AssertFatal(A, M, ...) if(!(A)) {printf(M, ##__VA_ARGS__); assert(A); }

extern int usrp_tx_thread;

typedef struct {

    // --------------------------------
    // variables for USRP configuration
    // --------------------------------
    //! USRP device pointer
    uhd::usrp::multi_usrp::sptr usrp;

    //create a send streamer and a receive streamer
    //! USRP TX Stream
    uhd::tx_streamer::sptr tx_stream;
    //! USRP RX Stream
    uhd::rx_streamer::sptr rx_stream;

    //! USRP TX Metadata
    uhd::tx_metadata_t tx_md;
    //! USRP RX Metadata
    uhd::rx_metadata_t rx_md;

    //! Sampling rate
    double sample_rate;

    //! TX forward samples. We use usrp_time_offset to get this value
    int tx_forward_nsamps; //166 for 20Mhz

    // --------------------------------
    // Debug and output control
    // --------------------------------
    int num_underflows;
    int num_overflows;
    int num_seq_errors;
    int64_t tx_count;
    int64_t rx_count;
    int wait_for_first_pps;
    int use_gps;
    //int first_tx;
    //int first_rx;
    //! timestamp of RX packet
    openair0_timestamp rx_timestamp;
} usrp_state_t;

/*! \brief Called to send samples to the USRP RF target
      @param device pointer to the device structure specific to the RF hardware target
      @param timestamp The timestamp at which the first sample MUST be sent
      @param buff Buffer which holds the samples
      @param nsamps number of samples to be sent
      @param antenna_id index of the antenna if the device has multiple antennas
      @param flags flags must be set to TRUE if timestamp parameter needs to be applied
*/
static int trx_usrp_write(openair0_device *device,
                          openair0_timestamp timestamp,
                          void **buff,
                          int nsamps,
                          int cc,
                          int flags) {
    int ret=0;
    usrp_state_t *s = (usrp_state_t *)device->priv;
    int nsamps2;  // aligned to upper 32 or 16 byte boundary

    int flags_lsb = flags&0xff;
    int flags_msb = (flags>>8)&0xff;

    int end;
    // openair0_thread_t *write_thread = &device->write_thread;
    // openair0_write_package_t *write_package = write_thread->write_package;

    AssertFatal( MAX_WRITE_THREAD_BUFFER_SIZE >= cc,"Do not support more than %d cc number\n", MAX_WRITE_THREAD_BUFFER_SIZE);

    boolean_t first_packet_state=false,last_packet_state=false;

    if (flags_lsb == 2) { // start of burst
        //      s->tx_md.start_of_burst = true;
        //      s->tx_md.end_of_burst = false;
        first_packet_state = true;
        last_packet_state  = false;
    } else if (flags_lsb == 3) { // end of burst
        //s->tx_md.start_of_burst = false;
        //s->tx_md.end_of_burst = true;
        first_packet_state = false;
        last_packet_state  = true;
    } else if (flags_lsb == 4) { // start and end
        //  s->tx_md.start_of_burst = true;
        //  s->tx_md.end_of_burst = true;
        first_packet_state = true;
        last_packet_state  = true;
    } else if (flags_lsb==1) { // middle of burst
        //  s->tx_md.start_of_burst = false;
        //  s->tx_md.end_of_burst = false;
        first_packet_state = false;
        last_packet_state  = false;
    }
    else if (flags_lsb==10) { // fail safe mode
        // s->tx_md.has_time_spec = false;
        // s->tx_md.start_of_burst = false;
        // s->tx_md.end_of_burst = true;
        first_packet_state = false;
        last_packet_state  = true;
    }

    if(usrp_tx_thread == 0){

    #if defined(__x86_64) || defined(__i386__)
    #ifdef __AVX2__
        nsamps2 = (nsamps+7)>>3;
        __m256i buff_tx[cc<2?2:cc][nsamps2];
    #else
        nsamps2 = (nsamps+3)>>2;
        __m128i buff_tx[cc<2?2:cc][nsamps2];
    #endif
    #elif defined(__arm__)
            nsamps2 = (nsamps+3)>>2;
        int16x8_t buff_tx[cc<2?2:cc][nsamps2];
    #else
        #error Unsupported CPU architecture, USRP device cannot be built
    #endif

        // bring RX data into 12 LSBs for softmodem RX
        for (int i=0; i<cc; i++) {
            for (int j=0; j<nsamps2; j++) {
#if defined(__x86_64__) || defined(__i386__)
                #ifdef __AVX2__
        buff_tx[i][j] = _mm256_slli_epi16(((__m256i *)buff[i])[j],4);
#else
        buff_tx[i][j] = _mm_slli_epi16(((__m128i *)buff[i])[j],4);
#endif
#elif defined(__arm__)
                buff_tx[i][j] = vshlq_n_s16(((int16x8_t *)buff[i])[j],4);
#endif
            }
        }

        s->tx_md.has_time_spec  = true;
        s->tx_md.start_of_burst = (s->tx_count==0) ? true : first_packet_state;
        s->tx_md.end_of_burst   = last_packet_state;
        s->tx_md.time_spec      = uhd::time_spec_t::from_ticks(timestamp, s->sample_rate);
        s->tx_count++;

        // bit 3 enables gpio (for backward compatibility)
        if (flags_msb&8) {
            // push GPIO bits 7-9 from flags_msb
            int gpio789=(flags_msb&7)<<7;
            s->usrp->set_command_time(s->tx_md.time_spec);
            s->usrp->set_gpio_attr("FP0", "OUT", gpio789, 0x380);
            s->usrp->clear_command_time();
        }

        if (cc>1) {
            std::vector<void *> buff_ptrs;

            for (int i=0; i<cc; i++)
                buff_ptrs.push_back(&(((int16_t *)buff_tx[i])[0]));

            ret = (int)s->tx_stream->send(buff_ptrs, nsamps, s->tx_md);
        }
        else {
            ret = (int)s->tx_stream->send(&(((int16_t *)buff_tx[0])[0]), nsamps, s->tx_md);
        }

        if (ret != nsamps) LOG_E(HW,"[xmit] tx samples %d != %d\n",ret,nsamps);
        return ret;
    }
    else{

        LOG_E(HW, "Only 1 thread transmit!\n");
        // pthread_mutex_lock(&write_thread->mutex_write);

        // if(write_thread->count_write >= MAX_WRITE_THREAD_PACKAGE){
        //     LOG_W(HW,"Buffer overflow, count_write = %d, start = %d end = %d, resetting write package\n", write_thread->count_write, write_thread->start, write_thread->end);
        //     write_thread->end = write_thread->start;
        //     write_thread->count_write = 0;
        // }

        // end = write_thread->end;
        // write_package[end].timestamp    = timestamp;
        // write_package[end].nsamps       = nsamps;
        // write_package[end].cc           = cc;
        // write_package[end].first_packet = first_packet_state;
        // write_package[end].last_packet  = last_packet_state;
        // write_package[end].flags_msb    = flags_msb;
        // for (int i = 0; i < cc; i++)
        //     write_package[end].buff[i]    = buff[i];
        // write_thread->count_write++;
        // write_thread->end = (write_thread->end + 1)% MAX_WRITE_THREAD_PACKAGE;
        // LOG_D(HW,"Signaling TX TS %llu\n",(unsigned long long)timestamp);
        // pthread_cond_signal(&write_thread->cond_write);
        // pthread_mutex_unlock(&write_thread->mutex_write);
        return -1;
    }

}

/*! \brief Receive samples from hardware.
 * Read \ref nsamps samples from each channel to buffers. buff[0] is the array for
 * the first channel. *ptimestamp is the time at which the first sample
 * was received.
 * \param device the hardware to use
 * \param[out] ptimestamp the time at which the first sample was received.
 * \param[out] buff An array of pointers to buffers for received samples. The buffers must be large enough to hold the number of samples \ref nsamps.
 * \param nsamps Number of samples. One sample is 2 byte I + 2 byte Q => 4 byte.
 * \param antenna_id Index of antenna for which to receive samples
 * \returns the number of sample read
*/
static int trx_usrp_read(openair0_device *device, openair0_timestamp *ptimestamp, void **buff, int nsamps, int cc) {
   usrp_state_t *s = (usrp_state_t *)device->priv;

  int samples_received=0;
  int nsamps2;  // aligned to upper 32 or 16 byte boundary
#if defined(__x86_64) || defined(__i386__)
#ifdef __AVX2__
  nsamps2 = (nsamps+7)>>3;
  __m256i buff_tmp[cc<2 ? 2 : cc][nsamps2];
#else
  nsamps2 = (nsamps+3)>>2;
  __m128i buff_tmp[cc<2 ? 2 : cc][nsamps2];
#endif
#elif defined(__arm__)
  nsamps2 = (nsamps+3)>>2;
  int16x8_t buff_tmp[cc<2 ? 2 : cc][nsamps2];
#endif

  int rxshift;
  switch (device->type) {
     case USRP_B200_DEV:
        rxshift=4;
        break;
     case USRP_X300_DEV:
     case USRP_N300_DEV:
        rxshift=2;
        break;
     default:
       AssertFatal(1==0,"Shouldn't be here\n");
  }

    samples_received=0;
    while (samples_received != nsamps) {

      if (cc>1) {
      // receive multiple channels (e.g. RF A and RF B)
        std::vector<void *> buff_ptrs;

        for (int i=0; i<cc; i++) buff_ptrs.push_back(buff_tmp[i]+samples_received);

        samples_received += s->rx_stream->recv(buff_ptrs, nsamps, s->rx_md);
      } else {
      // receive a single channel (e.g. from connector RF A)

        samples_received += s->rx_stream->recv((void*)((int32_t*)buff_tmp[0]+samples_received),
                                               nsamps-samples_received, s->rx_md);
      }
      if  ((s->wait_for_first_pps == 0) && (s->rx_md.error_code!=uhd::rx_metadata_t::ERROR_CODE_NONE))
        break;

      if ((s->wait_for_first_pps == 1) && (samples_received != nsamps)) {
        printf("sleep...\n"); //usleep(100);
      }
    }
    if (samples_received == nsamps) s->wait_for_first_pps=0;

    // bring RX data into 12 LSBs for softmodem RX
    for (int i=0; i<cc; i++) {
      for (int j=0; j<nsamps2; j++) {
#if defined(__x86_64__) || defined(__i386__)
#ifdef __AVX2__
        // FK: in some cases the buffer might not be 32 byte aligned, so we cannot use avx2

        if ((((uintptr_t) buff[i])&0x1F)==0) {
          ((__m256i *)buff[i])[j] = _mm256_srai_epi16(buff_tmp[i][j],rxshift);
        } else {
          ((__m128i *)buff[i])[2*j] = _mm_srai_epi16(((__m128i *)buff_tmp[i])[2*j],rxshift);
          ((__m128i *)buff[i])[2*j+1] = _mm_srai_epi16(((__m128i *)buff_tmp[i])[2*j+1],rxshift);
        }

#else
        ((__m128i *)buff[i])[j] = _mm_srai_epi16(buff_tmp[i][j],rxshift);
#endif
#elif defined(__arm__)
        ((int16x8_t *)buff[i])[j] = vshrq_n_s16(buff_tmp[i][j],rxshift);
#endif
      }
    }

    if (samples_received < nsamps) {
      LOG_E(HW,"[recv] received %d samples out of %d\n",samples_received,nsamps);
    }
  if ( s->rx_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
    LOG_E(HW, "%s\n", s->rx_md.to_pp_string(true).c_str());

  s->rx_count += nsamps;
  s->rx_timestamp = s->rx_md.time_spec.to_ticks(s->sample_rate);
  *ptimestamp = s->rx_timestamp;

  // push GPIO bits 7-9 from flags_msb
   /*s->usrp->set_command_time(uhd::time_spec_t::from_ticks((s->rx_timestamp+(2*nsamps)),s->sample_rate));
   s->usrp->set_gpio_attr("FP0", "OUT", gpio789<<7, 0x380);
   s->usrp->clear_command_time();
   gpio789 = (gpio789+1)&7;*/

//   if ( recPlay != NULL) { // record mode
//     // Copy subframes to memory (later dump on a file)
    
//     } else
//       exit_function(__FILE__, __FUNCTION__, __LINE__,"Recording reaches max iq limit\n");
//   }

  return samples_received;
}


/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // transmit variables to be set by po
    std::string tx_args, tx_file, tx_ant, tx_subdev, ref, otw, tx_channels;
    double tx_rate, tx_freq, tx_gain, wave_freq, tx_bw;
    float ampl;

    // receive variables to be set by po
    std::string rx_args, rx_file, type, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, spb;
    double rx_rate, rx_freq, rx_gain, rx_bw;
    double settling;

    std::string args = "addr=192.168.20.2";


    usrp_state_t *s;

    // 
    openair0_device *device;
    device = (openair0_device *)calloc(sizeof(openair0_device),1);
    //

    uhd::device_addrs_t device_adds = uhd::device::find(args);

    if (device_adds.size() == 0) {
      LOG_E(HW,"No USRP Device Found.\n ");
      free(s);
      return -1;
    } else if (device_adds.size() > 1) {
      LOG_E(HW,"More than one USRP Device Found. Please specify device more precisely in config file.\n");
      free(s);
      return -1;
    } else {
        LOG_I(HW,"USRP Device Found.\n");
        std::cout << args << std::endl;
    }

    if ( device->priv == NULL) {
      s=(usrp_state_t *)calloc(sizeof(usrp_state_t),1);
      device->priv=s;
      AssertFatal( s!=NULL,"USRP device: memory allocation failure\n");
    } else {
      LOG_E(HW, "multiple device init detected\n");
      return 0;
    }

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("tx-args", po::value<std::string>(&tx_args)->default_value(args), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value(args), "uhd receive device address args")
        ("tx-file", po::value<std::string>(&tx_file)->default_value("tx_usrp_samples.dat"), "name of the file to write binary samples to")
        ("tx-file", po::value<std::string>(&rx_file)->default_value("rx_usrp_samples.dat"), "name of the file to write binary samples to")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type in file: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("tx-rate", po::value<double>(&tx_rate), "rate of transmit outgoing samples")
        ("rx-rate", po::value<double>(&rx_rate), "rate of receive incoming samples")
        ("tx-freq", po::value<double>(&tx_freq), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq), "receive RF center frequency in Hz")
        ("tx-gain", po::value<double>(&tx_gain), "gain for the transmit RF chain")
        ("rx-gain", po::value<double>(&rx_gain), "gain for the receive RF chain")
        ("tx-ant", po::value<std::string>(&tx_ant), "transmit antenna selection")
        ("rx-ant", po::value<std::string>(&rx_ant), "receive antenna selection")
        ("tx-subdev", po::value<std::string>(&tx_subdev), "transmit subdevice specification")
        ("rx-subdev", po::value<std::string>(&rx_subdev), "receive subdevice specification")
        ("tx-bw", po::value<double>(&tx_bw), "analog transmit filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw), "analog receive filter bandwidth in Hz")
        // ("wave-type", po::value<std::string>(&wave_type)->default_value("CONST"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        // ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("tx-channels", po::value<std::string>(&tx_channels)->default_value("0"), "which TX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx-int-n", "tune USRP TX with integer-N tuning")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
    ;

    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << "UHD TXRX Loopback to File " << desc << std::endl;
        return ~0;
    }

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the transmit usrp device with: %s...") % tx_args
              << std::endl;
    s->usrp = uhd::usrp::multi_usrp::make(tx_args);
    std::cout << std::endl;
    // std::cout << boost::format("Creating the receive usrp device with: %s...") % rx_args
    //           << std::endl;
    // uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);

    // always select the subdevice first, the channel mapping affects the other settings
    // if (vm.count("tx-subdev"))
    //     tx_usrp->set_tx_subdev_spec(tx_subdev);
    // if (vm.count("rx-subdev"))
    //     rx_usrp->set_rx_subdev_spec(rx_subdev);

    // detect which channels to use
    std::vector<std::string> tx_channel_strings;
    std::vector<size_t> tx_channel_nums;
    boost::split(tx_channel_strings, tx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < tx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(tx_channel_strings[ch]);
        if (chan >= s->usrp->get_tx_num_channels()) {
            throw std::runtime_error("Invalid TX channel(s) specified.");
        } else
            tx_channel_nums.push_back(std::stoi(tx_channel_strings[ch]));
    }
    std::vector<std::string> rx_channel_strings;
    std::vector<size_t> rx_channel_nums;
    boost::split(rx_channel_strings, rx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < rx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(rx_channel_strings[ch]);
        if (chan >= s->usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid RX channel(s) specified.");
        } else
            rx_channel_nums.push_back(std::stoi(rx_channel_strings[ch]));
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        s->usrp->set_clock_source(ref);
    }

    std::cout << "Using RX_TX Device: " << s->usrp->get_pp_string() << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    s->usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (s->usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    s->usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (s->usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }

    for (size_t ch = 0; ch < tx_channel_nums.size(); ch++) {
        size_t channel = tx_channel_nums[ch];
        if (tx_channel_nums.size() > 1) {
            std::cout << "Configuring TX Channel " << channel << std::endl;
        }
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (tx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tx_tune_request(tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        s->usrp->set_tx_freq(tx_tune_request, channel);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (s->usrp->get_tx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the rf gain
        if (vm.count("tx-gain")) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain
                      << std::endl;
            s->usrp->set_tx_gain(tx_gain, channel);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % s->usrp->get_tx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                      << std::endl;
            s->usrp->set_tx_bandwidth(tx_bw, channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % s->usrp->get_tx_bandwidth(channel)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            s->usrp->set_tx_antenna(tx_ant, channel);
    }

    for (size_t ch = 0; ch < rx_channel_nums.size(); ch++) {
        size_t channel = rx_channel_nums[ch];
        if (rx_channel_nums.size() > 1) {
            std::cout << "Configuring RX Channel " << channel << std::endl;
        }

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t rx_tune_request(rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        s->usrp->set_rx_freq(rx_tune_request, channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (s->usrp->get_rx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the receive rf gain
        if (vm.count("rx-gain")) {
            std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain
                      << std::endl;
            s->usrp->set_rx_gain(rx_gain, channel);
            std::cout << boost::format("Actual RX Gain: %f dB...")
                             % s->usrp->get_rx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                      << std::endl;
            s->usrp->set_rx_bandwidth(rx_bw, channel);
            std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                             % (s->usrp->get_rx_bandwidth(channel) / 1e6)
                      << std::endl
                      << std::endl;
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            s->usrp->set_rx_antenna(rx_ant, channel);
    }

    // Align times in the RX USRP (the TX USRP does not require time-syncing)
    if (s->usrp->get_num_mboards() > 1) {
        s->usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    }

    // for the const wave, set the wave freq for small samples per period
    // if (wave_freq == 0 and wave_type == "CONST") {
    //     wave_freq = tx_usrp->get_tx_rate() / 2;
    // }

    // error when the waveform is not possible to generate
    // if (std::abs(wave_freq) > tx_usrp->get_tx_rate() / 2) {
    //     throw std::runtime_error("wave freq out of Nyquist zone");
    // }
    // if (tx_usrp->get_tx_rate() / std::abs(wave_freq) > wave_table_len / 2) {
    //     throw std::runtime_error("wave freq too small for table");
    // }

    // pre-compute the waveform values
    // const size_t step = std::lround(wave_freq / tx_usrp->get_tx_rate() * wave_table_len);
    size_t index = 0;

    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("sc16", otw);
    stream_args.channels             = tx_channel_nums;
    // uhd::tx_streamer::sptr tx_stream = s->usrp->get_tx_stream(stream_args);
    s->rx_stream = s->usrp->get_rx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    // if (spb == 0)
    //     spb = tx_stream->get_max_num_samps() * 10;
    // std::vector<std::complex<float>> buff(spb);
    // int num_channels = tx_channel_nums.size();

    // setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(0.5); // give us 0.5 seconds to fill the tx buffers

    // Check Ref and LO Lock detect
    std::vector<std::string> tx_sensor_names, rx_sensor_names;
    tx_sensor_names = s->usrp->get_tx_sensor_names(0);
    if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
        != tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = s->usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    rx_sensor_names = s->usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = s->usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    tx_sensor_names = s->usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = s->usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = s->usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    rx_sensor_names = s->usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = s->usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = s->usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    s->usrp->set_time_now(uhd::time_spec_t(0.0));

    // start transmit worker thread
    // std::thread transmit_thread([&]() {
        // transmit_worker(buff, wave_table, tx_stream, md, step, index, num_channels);
    // });

    // recv to file
    // if (type == "double")
    //     recv_to_file<std::complex<double>>(
    //         rx_usrp, "fc64", otw, rx_file, spb, total_num_samps, settling, rx_channel_nums);
    // else if (type == "float")
    //     recv_to_file<std::complex<float>>(
    //         rx_usrp, "fc32", otw, rx_file, spb, total_num_samps, settling, rx_channel_nums);
    // else
    if (type == "short") {
                // recv_to_file<std::complex<short>>(
        //     rx_usrp, "sc16", otw, rx_file, spb, total_num_samps, settling, rx_channel_nums);

        openair0_timestamp timestamp;
        int cc = 1;
        void *rxp[cc];
        int nb_slot_frame = 10;
        int nsamps = 30720;
        int32_t rxdata[nsamps*nb_slot_frame + 2048];

        int slot_nr = 0;
        int absolute_slot = 0;
        
        int recv_samps= 30720;

        device->type = USRP_N300_DEV;

        while(1) {

            for (int i = 0; i < cc; i++) {
                rxp[i] = (void *)&rxdata[slot_nr*30720];
            }
            
            recv_samps = trx_usrp_read(device, &timestamp, (void **)rxp, nsamps, cc);
            //openair0_device *device, openair0_timestamp *ptimestamp, void **buff, int nsamps, int cc
            AssertFatal(nsamps == recv_samps, "30720 != nsamps");
            
            // trx_usrp_write();
            slot_nr == nb_slot_frame ? 0 : slot_nr++;
            absolute_slot++;
            if (absolute_slot % 256 == 0)
                LOG_I(PHY, "absolute_slot = %d slot_nr %d\n", absolute_slot, slot_nr);
            
        }

    } else {
        // clean up transmit worker
        stop_signal_called = true;
        // transmit_thread.join();
        throw std::runtime_error("Unknown type " + type);
    }

    // clean up transmit worker
    stop_signal_called = true;
    // transmit_thread.join();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}