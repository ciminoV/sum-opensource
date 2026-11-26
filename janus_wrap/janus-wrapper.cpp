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

#include <janus-wrapper.h>
#include <string.h>
#include <string>
#include <climits>
#include <janus/dump.h>

const unsigned int JanusWrapper::JANUS_RED_SIZE = 1024;

namespace {
constexpr unsigned int CRC_SIZE = 2;          // CRC-16 bytes appended by 016-01
constexpr unsigned TX_MAX_CARGO_BYTES = 480;  // largest cargo of any supported plugin
constexpr int TX_READ_BLOCK_FRAMES = 8192;    // frames copied per FIFO read

// Exact number of frames janus_tx_execute() writes for one packet carrying
// cargo_bytes of cargo.
// (Mirrors janus tx.c, modulator.c and wake_up_tones.c)
unsigned long txFrameCount(janus_simple_tx_t tx, const janus_parameters_t params,
                           unsigned cargo_bytes)
{
    const janus_ostream_t os = janus_simple_tx_get_ostream(tx);
    const janus_real_t chip_dur = janus_simple_tx_get_pset(tx)->chip_dur;

    // Cargo bits: data plus 1-byte tail that flushes the convolutional encoder
    const unsigned long cargo_bits =
        cargo_bytes ? (cargo_bytes + JANUS_CONV_MEM_SIZE) * 8UL : 0;

    // Chips: 32 uncoded preamble chips, 8-byte baseline packet and
    // the cargo, each with its tail.
    const unsigned long chips = JANUS_PREAMBLE_CHIP_COUNT +
        JANUS_ALPHABET_SIZE * (JANUS_MIN_PKT_SIZE + JANUS_CONV_MEM_SIZE) * 8UL +
        JANUS_ALPHABET_SIZE * cargo_bits;

    // Chips to samples
    unsigned long written_samples = (unsigned long) JANUS_ROUND(chips * (chip_dur * os->fs));

    // Silence before and after the frame: 5 chips of silence each side.
    if (params->pad) {
        const unsigned long pad = 2UL *
            (unsigned)(JANUS_FMAX(
                JANUS_ROUND(chip_dur / os->ts),
                JANUS_ROUND(chip_dur * os->fs))
            * JANUS_SILENCE_CHIP_COUNT);

        written_samples += pad;
    }

    // Wake-up tones: 3 tones of 4 chips each, then a 0.4 s silent gap.
    if (params->wut) {
        const unsigned long tones = 3UL *
            (unsigned)(JANUS_WAKE_UP_TONE_CHIP_COUNT *
                (JANUS_ROUND(chip_dur / os->ts) + 1));

        written_samples += tones + (unsigned)(JANUS_WUT_GAP * os->fs);
            * JANUS_SILENCE_CHIP_COUNT);
    }

    // Final silence so the length is a multiple of stream_mul.
    if (os->multiple > 1 && written_samples % os->multiple)
        written_samples += os->multiple - written_samples % os->multiple;

    return written_samples;
}

const char* field(janus_app_fields_t fields, const char* name)
{
    for (unsigned i = 0; i < fields->field_count; ++i)
        if (strcmp(fields->fields[i].name, name) == 0)
            return fields->fields[i].value;

    return nullptr;
}

bool number(const char* s, unsigned max, unsigned& n)
{
    n = 0;

    if (!s || !*s) return false;

    for (; *s; ++s) {
        if (*s < '0' || *s > '9')
			return false;

        n = n * 10 + (*s - '0');

        if (n > max)
			return false;
    }

    return true;
}
}

JanusWrapper::JanusWrapper()
:
  params(nullptr),
  simple_tx(nullptr),
  state_tx(nullptr),
  simple_rx(nullptr),
  state_rx(nullptr),
  carrier_sensing(nullptr),
  rx_buffer(nullptr),
  active(false),
  demod_thread(),
  mutex_demod(),
  cv_demod(),
  demod_data_available(0),
  mac_mode(0),
  class_id(16),
  app_type(0)
  chat_coding(0),
  fifo_arg("1"),
  tx_fifo_frames()
{
  params = janus_parameters_new();
  initDefaultParams();
}


JanusWrapper::~JanusWrapper()
{
  active = false;
  cv_demod.notify_one();
  if(demod_thread.joinable())
    demod_thread.join();
  janus_parameters_free(params);
  if(simple_tx)
    janus_simple_tx_free(simple_tx);
  if(simple_rx)
    janus_simple_rx_free(simple_rx);
  if(state_tx)
    janus_tx_state_free(state_tx);
  if(state_rx)
    janus_rx_state_free(state_rx);
  if(carrier_sensing)
    janus_carrier_sensing_free(carrier_sensing);
}

bool 
JanusWrapper::janusSetup() 
{
  // Build a temporary transmitter to get the pset and stream JANUS resolve,
  // then size the FIFO for the longest frame this wrapper can transmit.
  janus_simple_tx_t probe = janus_simple_tx_new(params);
  if (!probe) 
		return false;

  const unsigned long frame_size =
		janus_ostream_get_frame_size(janus_simple_tx_get_ostream(probe));
  const unsigned long need =
      (txFrameCount(probe, params, TX_MAX_CARGO_BYTES) + 1) * frame_size;

  janus_simple_tx_free(probe);

  unsigned long capacity = 1;
  while (capacity < need) 
		capacity <<= 1; // the JANUS FIFO rounds up the same way

  if (capacity > INT_MAX)
		return false;

  tx_fifo_frames = capacity / frame_size;

  fifo_arg = std::to_string(capacity);
  params->stream_driver_args = fifo_arg.c_str();

  simple_tx = janus_simple_tx_new(params);
  simple_rx = janus_simple_rx_new(params);

  if (!simple_tx || !simple_rx)
		return false;

  state_tx = janus_tx_state_new((params->verbose > 1));
  state_rx = janus_rx_state_new(params);
  carrier_sensing = janus_carrier_sensing_new(janus_simple_rx_get_rx(simple_rx));
  active = true;
  demod_thread = std::thread(&JanusWrapper::performDemodulation,this);

  return true;
}

void 
JanusWrapper::initDefaultParams()
{
  params->verbose = 0;
  params->pset_id = 0;
  params->pset_center_freq = 11520;
  params->pset_bandwidth = 4160;
  params->stream_channel = 0;
  params->stream_channel_count = 1;
  params->stream_passband = 1;
  params->wut = 0;
  params->stream_driver = "fifo";
  params->stream_driver_args = fifo_arg.c_str();
  params->stream_fs = 44100;
  params->stream_format = "S16";
  params->stream_passband = 1;
  params->stream_amp = JANUS_REAL_CONST(0.95);
  params->stream_mul = 1;
  params->doppler_correction = 1;
  params->doppler_max_speed = JANUS_REAL_CONST(5.0);
  params->compute_channel_spectrogram = 1;
  params->detection_threshold = JANUS_REAL_CONST(2.5);
  params->colored_bit_prob = 0;
  params->cbp_high2medium  = JANUS_REAL_CONST(0.2);
  params->cbp_medium2low   = JANUS_REAL_CONST(0.35);
}
void 
JanusWrapper::initPacket(janus_packet_t packet)
{
  janus_packet_set_mobility(packet, 0);
  janus_packet_set_tx_rx(packet, 1);
  janus_packet_set_forward(packet, 0);
  janus_packet_set_class_id(packet, class_id);
  janus_packet_set_app_type(packet, app_type);
  janus_packet_set_validity(packet, 1); 
}

bool
JanusWrapper::setJanusClassID(unsigned int id)
{
	// Supported class id: 16; 11
    if (id != 16 && id != 11)
        return false;

    class_id = static_cast<janus_uint8_t>(id);
    return true;
}

bool
JanusWrapper::setJanusApplicationType(unsigned int type)
{
	// Supported plugins: 16_00; 16_01; 11_01; 11_02
    if ((class_id == 16 && (type != 0 && type != 1)) || 
		(class_id == 11 && (type != 1 && type != 2)))
        return false;

    app_type = static_cast<janus_uint8_t>(type);
    return true;
}

bool
JanusWrapper::setChatCoding(unsigned coding)
{
    if (active || coding > 3 || (class_id == 11 && app_type == 2 && coding > 1))
        return false;
    chat_coding = coding;
    return true;
}

bool
JanusWrapper::setCarrierFrequency(float carr_freq)
{
  if(carr_freq != 0.0) {
    params->pset_center_freq = carr_freq;
    return true;
  }
  return false;
}

bool
JanusWrapper::setBandwidth(float bw)
{
  if(bw > 0.0 && bw <= params->pset_center_freq) 
  {
    params->pset_bandwidth = bw;
    return true;
  }
  return false;
}

void 
JanusWrapper::setVerbose(int verbose)
{
  params->verbose = verbose;
}

void 
JanusWrapper::setWakeUpTones(int wut)
{
  params->wut = wut;
}

void
JanusWrapper::setMacMode(int _mac_mode){
  mac_mode = _mac_mode;
}

void
JanusWrapper::setSamplingFrequency(int sampling_frequency_)
{
  if(sampling_frequency_ != 0.0) {
    params->stream_fs = sampling_frequency_;
  }
}


bool
JanusWrapper::setRxBuffer(std::shared_ptr<ConcBuffer> buffer)
{
  if(buffer == nullptr) {
    return false;
  }

  rx_buffer = buffer;
  return true;
}

void
JanusWrapper::setTxGain(float gain)
{
  if(gain > 0.0f && gain < 1.0f)
    params->stream_amp = JANUS_REAL_CONST(gain);
  else
    return;
}

void JanusWrapper::pushRxChunk(std::shared_ptr<Chunk> rx_chunk)
{
  rx_buffer->push(rx_chunk);
}

bool
JanusWrapper::modulate(std::shared_ptr<Chunk> chunk)
{
    if (!simple_tx || !chunk || chunk->getSize() == 0) {
        return false;
    }

    const bool chat = class_id == 11 && (app_type == 1 || app_type == 2);
    if (!chat && !(class_id == 16 && app_type <= 1)) 
		return false;

    const size_t header_size = chunk->getHeaderSize();
    const size_t payload_size = chunk->getSize();

    if (payload_size > JANUS_MAX_PKT_CARGO_SIZE ||
        (!chat && header_size > JANUS_MAX_PKT_CARGO_SIZE - payload_size))
        return false;

    if (chat && (header_size != (mac_mode ? sizeof(MacHdr) : 0) ||
                 (app_type == 2 && chat_coding > 1)))
        return false;

    janus_packet_t packet = janus_packet_new(params->verbose);
    initPacket(packet);

    const size_t prefix = chat ? 0 : header_size;
    std::vector<janus_uint8_t> data(prefix + payload_size);
    if (prefix)
        memcpy(data.data(), chunk->getHeader(), header_size);
    memcpy(data.data() + prefix, chunk->data(), payload_size);

	// Plugin 11_01 or 11_02
    if (chat) {
        janus_packet_set_version(packet, 4);
        janus_packet_set_chip_duration(packet, janus_simple_tx_get_pset(simple_tx)->chip_dur);

        unsigned station = 0, destination = 255;

        if (mac_mode) {
            MacHdr hdr{};
            memcpy(&hdr, chunk->getHeader(), sizeof hdr);
            station = hdr.src_addr;
            destination = hdr.dest_addr == BROADCAST_ADDRESS ? 255U : hdr.dest_addr;
        }

        auto fields = janus_app_fields_new();
        janus_app_fields_add_field(fields, "Station Identifier", std::to_string(station).c_str());
        janus_app_fields_add_field(fields, "Destination Identifier", std::to_string(destination).c_str());
        janus_app_fields_add_field(fields, "Coding", std::to_string(chat_coding).c_str());
        janus_packet_set_application_data_fields(packet, fields);
        janus_app_fields_free(fields);
    }

	// Copy payload into packet cargo
    int cargo_error = janus_packet_set_cargo(
        packet, data.data(), static_cast<unsigned>(data.size()));

	// Plugin 16_01 or 11_01
    if (cargo_error == JANUS_ERROR_NONE && (chat || app_type == 1))
        cargo_error = janus_packet_encode_cargo_byte(packet);

	// Plugin 11_02
    if (cargo_error == JANUS_ERROR_NONE && chat && app_type == 2)
        cargo_error = janus_packet_set_tx_reservation_cargo(
            packet, janus_packet_get_desired_cargo_size(packet));

    if (cargo_error != JANUS_ERROR_NONE ||
        janus_packet_encode_application_data(packet) != JANUS_ERROR_NONE) {
        janus_packet_free(packet);
        return false;
    }

    if (janus_packet_get_cargo_size(packet) > TX_MAX_CARGO_BYTES) {
        janus_packet_free(packet);
        return false;
    }

    janus_packet_set_validity(packet, 2);

    if (janus_simple_tx_execute(simple_tx, packet, state_tx) < 0) {
        janus_packet_free(packet);
        return false;
    }

    janus_tx_state_dump(state_tx);
    janus_packet_dump(packet);

    janus_ostream_t ostream = janus_simple_tx_get_ostream(simple_tx);

    auto samples = std::make_shared<Samples>(0);
    int16_t block[TX_READ_BLOCK_FRAMES];
    int frames;

    while ((frames = ostream->read(ostream, block, TX_READ_BLOCK_FRAMES)) > 0)
        samples->samples.insert(samples->samples.end(), block, block + frames);
    samples->sample_size = samples->samples.size();

    // A full FIFO may have silently discarded the tail: don't transmit it.
    if (frames < 0 || !samples->sample_size ||
        samples->sample_size >= tx_fifo_frames) {
        janus_packet_free(packet);
        return false;
    }

    chunk->setModulatedSamples(samples);

    janus_packet_free(packet);
    return true;
}

bool 
JanusWrapper::demodulate(std::shared_ptr<Samples> samples)
{
  if (!simple_rx || !samples || samples->sample_size > samples->samples.size() ||
      samples->sample_size > INT_MAX / sizeof(int16_t)) return false;

  janus_istream_t istream = janus_simple_rx_get_istream(simple_rx);
  std::unique_lock<std::mutex> lk(mutex_demod);
  int accepted = istream->write(istream,samples->samples.data(),samples->sample_size);

  if (accepted < 0)
		return false;

  demod_data_available += accepted;

  if(demod_data_available >= 10) {
    cv_demod.notify_one();
  }

  lk.unlock();  

  return static_cast<size_t>(accepted) == samples->sample_size;
}

void JanusWrapper::deliverPacket(janus_packet_t packet)
{
    const auto cls = janus_packet_get_class_id(packet);
    const auto app = janus_packet_get_app_type(packet);
    if (!rx_buffer || cls != class_id || app != app_type ||
        janus_packet_get_validity(packet) != 2 ||
        janus_packet_get_cargo_error(packet) != JANUS_ERROR_NONE) 
		return;

    MacHdr hdr{};
    std::shared_ptr<Chunk> out;
    if (cls == 11 && (app == 1 || app == 2)) {
        auto fields = janus_app_fields_new();
        janus_packet_get_application_data_fields(packet, fields);
        const char* text = field(fields, "Payload");
        unsigned len, src, dst;
        bool ok = text && number(field(fields, "Payload Size"), JANUS_MAX_PKT_CARGO_SIZE * 2, len)
                       && strlen(text) == len;
        if (ok && mac_mode) {
            ok = number(field(fields, "Station Identifier"), 15, src) &&
                 number(field(fields, "Destination Identifier"), 255, dst) &&
                 (dst == 255 || (dst >= 1 && dst <= 15));
            if (ok) {
                hdr.src_addr = src;
                hdr.dest_addr = dst == 255 ? BROADCAST_ADDRESS : dst;
            }
        }
        if (ok) {
            out = std::make_shared<Chunk>(len);
            if (len) memcpy(out->data(), text, len);
            out->setSize(len);
        }
        janus_app_fields_free(fields);
    } else if (cls == 16 && app <= 1) {
        const auto* data = janus_packet_get_cargo(packet);
        unsigned len = janus_packet_get_cargo_size(packet);
        if (app == 1) {
            if (len < CRC_SIZE) return;
            len -= CRC_SIZE;
        }
        if (!data || len <= (mac_mode ? sizeof hdr : 0)) return;
        if (mac_mode) {
            memcpy(&hdr, data, sizeof hdr);
            data += sizeof hdr;
            len -= sizeof hdr;
        }
        out = std::make_shared<Chunk>(len);
        memcpy(out->data(), data, len);
        out->setSize(len);
    }

    if (out && (!mac_mode || out->addHeader(&hdr, sizeof hdr)))
		pushRxChunk(out);
}

void
JanusWrapper::performDemodulation()
{
    unsigned queried_detection_time = 0;
    janus_packet_t packet_rx = janus_packet_new(params->verbose);
    janus_real_t time;

    initPacket(packet_rx);

    while (active.load()) {
        std::unique_lock<std::mutex> lck{mutex_demod};

        cv_demod.wait(lck, [&]() {
            return demod_data_available >= JanusWrapper::JANUS_RED_SIZE ||
                   !active.load();
        });

        if (!active.load())
            break;

        int rv = janus_simple_rx_execute(simple_rx, packet_rx, state_rx);

        if (rv < 0) {
            lck.unlock();

            if (rv == JANUS_ERROR_OVERRUN) {
                // Handle stream overrun.
            }

        } else if (rv > 0) {
            demod_data_available -= JanusWrapper::JANUS_RED_SIZE;

            if (demod_data_available < 0)
                demod_data_available = 0;

            lck.unlock();

            deliverPacket(packet_rx);

            janus_packet_reset(packet_rx);
            queried_detection_time = 0;
            janus_carrier_sensing_reset(carrier_sensing);

        } else {
            demod_data_available -= JanusWrapper::JANUS_RED_SIZE;

            if (demod_data_available < 0)
                demod_data_available = 0;

            lck.unlock();

            if (janus_simple_rx_has_detected(simple_rx) &&
                !queried_detection_time) {
                queried_detection_time = 1;
            }

            if (janus_carrier_sensing_execute(carrier_sensing, &time) > 0) {
                janus_carrier_sensing_window_power(carrier_sensing);
                janus_carrier_sensing_background_power(carrier_sensing);
            }
        }
    }

    janus_packet_free(packet_rx);
}
