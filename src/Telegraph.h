#ifndef TELEGRAPH_H
#define TELEGRAPH_H

#include <Arduino.h>
#include <assert.h>

#ifndef MAX_LISTENERS
#define MAX_LISTENERS 6
#endif

#ifndef BUFFER_MAX_SIZE
//* Must be a power of two!
#define BUFFER_MAX_SIZE 16
#endif

#ifndef MIN_AVAILABLE_DELTA
//* The minimum time that RX must be held to be considered available
#define MIN_AVAILABLE_DELTA 500000
#endif

namespace TelegraphUtils
{
    class list
    {
    private:
        unsigned short start;
        unsigned short stop;
        byte data[BUFFER_MAX_SIZE];

    public:
        list()
        {
            start = 0;
            stop = 0;
        }

        unsigned short size()
        {
            return abs(stop - start);
        }

        byte pop()
        {
            assert(start != stop);
            return data[index(start++)];
        }

        byte peek()
        {
            assert(start != stop);
            return data[index(start)];
        }

        void push(byte value)
        {
            assert(size() < BUFFER_MAX_SIZE);
            data[index(stop++)] = value;
        }

        void clear()
        {
            start = 0;
            stop = 0;
        }

        unsigned short index(unsigned short i)
        {
            return i & (BUFFER_MAX_SIZE - 1);
        }
    };
}

typedef struct
{
    unsigned long time;
    bool activated;
    bool mid_activated;
    bool available;
    byte val;
    unsigned short id;
    bool previous_reading;
    unsigned short n_bits_read;
} channel_t;

namespace master
{
    using namespace TelegraphUtils;

    class Telegraph
    {
    private:
        int rx_pins[MAX_LISTENERS];
        int tx_pin;
        unsigned short n_listeners = 0;
        list buffers[MAX_LISTENERS];
        channel_t channels[MAX_LISTENERS];
        unsigned int freq;
        unsigned int delta_us;

        void begin_transmission();
        void end_transmission();
        void transmit_byte(byte data);
        void reset_channel(unsigned short id);

    public:
        Telegraph(int tx_pin);
        short listen(int rx_pin);
        void begin(unsigned int frequency);
        byte read(unsigned short id);
        byte peek(unsigned short id);

        /**
         * @brief Blocking function
         *
         * Writes data to the connection at id
         */
        void write(byte *data, unsigned int size);
        int buff_size(unsigned short id);
        bool available(unsigned short id);
        void await(unsigned short id);
        void await_all();

        /**
         * @brief Reads all incoming data and place it into the data buffers
         */
        void tick();
    };

} // namespace master

namespace client
{
    using namespace TelegraphUtils;

    class Telegraph
    {
    private:
        int rx_pin;
        int tx_pin;
        list buffer;
        channel_t channel;
        unsigned int freq;
        unsigned int delta_us;

        void begin_transmission();
        void end_transmission();
        void transmit_byte(byte data);
        void wait_activation(unsigned int min_delay_ms);
        void reset_channel();

    public:
        Telegraph(int rx_pin, int tx_pin);
        void begin(unsigned int frequency);
        byte read();
        byte peek();
        void write(byte *data, unsigned int size);
        int buff_size();
        bool available();
        void await();
        void tick();
        void recv(unsigned short size);
    };
} // namespace client

#endif