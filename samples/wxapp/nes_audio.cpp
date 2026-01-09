#include "nes_audio.hpp"
#include "shapones/shapones.hpp"

namespace nes_audio {

static pa_simple *pa;
static std::thread *thread;
static volatile bool continue_flag;
static float dc_level = 0.0f;

static void play_thread_func();

void play() {
    int pa_errno;

    if (thread) stop();

    pa_sample_spec ss;
    ss.format = PA_SAMPLE_S16LE;
    ss.rate = 22050;
    ss.channels = 1;

    pa_buffer_attr ba;
    ba.maxlength = CHUNK_SIZE * 4;
    ba.tlength = CHUNK_SIZE;
    ba.prebuf = -1;
    ba.minreq = 64;

    pa = pa_simple_new(NULL, "ShapoNES", PA_STREAM_PLAYBACK, NULL, "play", &ss, NULL, &ba, &pa_errno);
    if (pa == NULL) {
        fprintf(stderr, "ERROR: Failed to connect pulseaudio server: %s\n", pa_strerror(pa_errno));
        return;
    }

    nes::apu::set_sampling_rate(ss.rate);

    thread = new std::thread(play_thread_func);

    continue_flag = true;
}

void stop() {
    if ( ! thread) return;
    continue_flag = false;
    thread->join();
    delete thread;
    thread = nullptr;
}

static void play_thread_func() {
    int pa_result, pa_errno;
    uint8_t buffer_u8[CHUNK_SIZE ];
    int16_t buffer_s16[CHUNK_SIZE];

    printf("Audio stream opened.\n");

    while (continue_flag) {
        nes::apu::service(buffer_u8, CHUNK_SIZE);
        for (int i = 0; i < CHUNK_SIZE; i++) {
            // Convert u8 sample [0,255] to s16 sample [-32768,32767] with DC offset removal
            float sample = buffer_u8[i] * 256.0f;
            dc_level = dc_level * 0.999f + sample * 0.001f;
            sample -= dc_level;
            if (sample > 32767.0f) sample = 32767.0f;
            if (sample < -32768.0f) sample = -32768.0f;
            buffer_s16[i] = (int16_t)sample;
        }
        pa_result = pa_simple_write(pa, buffer_s16, CHUNK_SIZE * sizeof(int16_t), &pa_errno);
        if (pa_result < 0) {
            fprintf(stderr, "ERROR: Failed to write data to pulseaudio: %s\n", pa_strerror(pa_errno));
            break;
        }
    }

    pa_simple_free(pa);

    printf("Audio stream closed.\n");
}

}
