/**
 * [2024] SubSeaPulse SRL 
 * All Rights Reserved.
 */

// This is free software: you can redistribute it and/or modify it        *
// under the terms of the GNU General Public License version 3 as         *
// published by the Free Software Foundation.                             *
//                                                                        *
// This program is distributed in the hope that it will be useful, but    *
// WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     *
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for       *
// more details.                                                          *
//                                                                        *
// You should have received a copy of the GNU General Public License      *
// along with this program. If not, see <http://www.gnu.org/licenses/>.   *


/**
 * @file janus_wrapper.h
 * @author Filippo Campagnaro
 * @version 1.0.0
 * @brief .
 */

#ifndef JANUS_WRAPPER_H
#define JANUS_WRAPPER_H


#include <thread>
#include <atomic>
#include <chunk.h>
#include <mutex>
#include <condition_variable>
#include <janus/janus.h>
#include <janus/defaults.h>

/**
 * .
 */
class JanusWrapper 
{
public:

  /**
  * Class constructor.
  */
  JanusWrapper();

  /**
  * Class destructor. It takes care on destroying all liquid-dsp objects.
  */
  ~JanusWrapper();

  /**
  * Function used to modulate a chunk.
  *
  * @param chunk shared pointer of the chunk to be modulated.
  * @return bool with value equal to the result of the function.
  */
  bool modulate(std::shared_ptr<Chunk> chunk);

  /**
  * Function used to demodulate a vector struct of samples.
  *
  * @param samples shared pointer of the samples struct to be demodulated.
  * @return bool with value equal to the result of the function.
  */
  bool demodulate(std::shared_ptr<Samples> samples);

  /**
  * Function used to set the pointer for the RX buffer.
  *
  * @return bool with value equal to the result of the function.
  */
  bool setRxBuffer(std::shared_ptr<ConcBuffer> buffer);

  bool setJanusApplicationType(unsigned int type);

  /**
  * Function used to set the carrier frequency
  *
  * @param carr_freq float of the carrier frequency, in Hz.
  * @return bool true if set is successful.
  */
  bool setCarrierFrequency(float carr_freq);

  /**
  * Function used to set the bandwidth.
  *
  * @param carr_freq float of the bandwidth in Hz.
  * @return bool true if set is successful.
  */
  bool setBandwidth(float bw);

  /**
  * Function used to set the sampling frequency.
  *
  * @param samples_per_symbol_ int of the sampling frequency.
  */
  void setSamplingFrequency(int sampling_frequency_);

  /**
  * Function used to set the tx_gain for transmission.
  *
  * @param gain float in range [0.0, 1.0].
  */
  void setTxGain(float gain);

  /**
  * Function used to set the verbose janus parameter.
  *
  * @param verbose integer in range [0, 2] (from less verbose to more verbose).
  */
  void setVerbose(int verbose);

  /**
  * Function used to enable/disable wake up tones.
  *
  * @param wut integer equal to 1 for enabling wake up tones, 0 for disabling them.
  */
  void setWakeUpTones(int wut);

  /**
  * Function used to enable/disable MAC mechanics.
  *
  * @param wut integer equal to 1 for enabling MAC support, 0 for disabling it.
  */
  void setMacMode(int _macmode);

  /**
  * Function used to create all the required janus objects.
  *
  * @return bool with value equal to the result of the function.
  */
  bool janusSetup();

private:

  /**
  * function used to initialize the janus packet.
  * 
  * @param packet pointer to the janus packet
  */
  void initPacket(janus_packet_t packet);

  /**
  * function used to push a packet to the rx buffer.
  * 
  * @param rx_chunk chunk received and correctly decoded
  */
  void pushRxChunk(std::shared_ptr<Chunk> rx_chunk);

  /**
  * function used to init the default Janus parameters.
  */
  void initDefaultParams();

  /**
  * function that performs the demodulation in a separated thread.
  */
  void performDemodulation();

  janus_parameters_t params; /**< Janus parameters*/

  janus_simple_tx_t simple_tx; /**< Janus transmitter */
  janus_tx_state_t state_tx;  /**< Janus transmitter state */

  janus_simple_rx_t simple_rx; /**< Janus receiver */
  janus_rx_state_t state_rx;  /**< Janus receiver state */
  janus_carrier_sensing_t carrier_sensing; /**< Janus carrier sensing */
  std::shared_ptr<ConcBuffer> rx_buffer; /**< Shared pointer to rx buffer in PHY */
  
  std::atomic<bool> active; /**< boolean flag to join the thread */
  std::thread demod_thread; /**< Thread that actually computes the demodulation */

  std::mutex mutex_demod; /**< mutex for demodulating */
  /** condition variable to wait for data to be demodulated */
  std::condition_variable cv_demod; 

  int demod_data_available; /**< boolean data samples inserted*/

  float carrier_freq; /**< Carrier frequency for framegenerator, in Hz */

  unsigned int samples_per_symbol; /**< Number of samples for a single symbol */

  float sampling_frequency; /**< Sampling frequency, in Hz */

  float bandwidth; /**< Bandwidth in Hz */

  static const unsigned int JANUS_RED_SIZE; /**< constant equal to JANUS DEFAULT_DRV_IN_SIZE */
  static const unsigned int MAX_SIZE_TO_READ; /**< max size to read from buffer */

  int mac_mode; /**< MAC mode, 0 is disabled, 1 enabled */

  janus_uint8_t app_type;
};

#endif
