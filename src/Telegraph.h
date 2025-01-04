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
    /**
     * Implementation of a queue, using a fixed size array for better efficiency
     */
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
        /**
         * @brief Constructor for a Telgraph instance
         * @param tx_pin The TX pin which all the modules listen to in parallel
         */
        Telegraph(int tx_pin);

        /**
         * @brief Listen to a specific pins, and register it as a module
         * @return The ID of the connection (used to identify the buffer, the pin...)
         */
        short listen(int rx_pin);

        /**
         * @brief Tell the module that the master is ready, and sets the baud rate
         * @param frequency Baud rate
         */
        void begin(unsigned int frequency);

        /**
         * @brief Read the first byte from the buffer (and discard it from the buffer)
         * @param id The ID of the buffer (returned by `Telegraph::listen()`)
         * @return The byte read
         */
        byte read(unsigned short id);

        /**
         * @brief Read the first byte in the buffer, and keep it in the buffer
         * @param id
         * @return The byte peeked
         */
        byte peek(unsigned short id);

        /**
         * @brief Write a serie of bytes to the TX pin
         * @param data A pointer to the array of byte to be written
         * @param size The size of the array
         * @warning Blocking function
         */
        void write(byte *data, unsigned int size);

        /**
         * @brief Get the logical size of the data buffer
         * @param id The ID of the buffer
         * @return The logical size of the buffer
         */
        unsigned short buff_size(unsigned short id);

        /**
         * @brief Check wether a channel is available for communication (is the module ready?)
         * @param id The ID of the channel
         * @return A bool
         */
        bool available(unsigned short id);

        /**
         * @brief Wait for a specific channel to be ready
         * @param id The ID of the channel
         * @warning Blocking function
         */
        void await(unsigned short id);

        /**
         * @brief Waits for all the registered channels to be ready
         * @warning Blocking function
         */
        void await_all();

        /**
         * @brief Reads all incoming data and place it into the data buffers if there is some
         * @warning To be called as often as ready. If heavy useage of blocking function is made, decreasing the baud rate can solve the problem
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
        /**
         * @brief Constructor for a Telgraph instance
         * @param tx_pin The TX pin which all the modules listen to in parallel
         */
        Telegraph(int rx_pin, int tx_pin);

        /**
         * @brief Tell the module that the master is ready, and sets the baud rate
         * @param frequency Baud rate
         */
        void begin(unsigned int frequency);

        /**
         * @brief Read the first byte from the buffer (and discard it from the buffer)
         * @return The byte read
         */
        byte read();

        /**
         * @brief Read the first byte in the buffer, and keep it in the buffer
         * @return The byte peeked
         */
        byte peek();

        /**
         * @brief Write a serie of bytes to the TX pin
         * @param data A pointer to the array of byte to be written
         * @param size The size of the array
         * @warning Blocking function
         */
        void write(byte *data, unsigned int size);

        /**
         * @brief Get the logical size of the data buffer
         * @return The logical size of the buffer
         */
        unsigned short buff_size();

        /**
         * @brief Check wether the channel is available for communication (is the master ready?)
         * @return A bool
         */
        bool available();

        /**
         * @brief Wait for the channel to be ready
         * @warning Blocking function
         */
        void await();

        /**
         * @brief Reads all incoming data and place it into the data buffer if there is some
         * @warning To be called as often as ready. If heavy useage of blocking function is made, decreasing the baud rate can solve the problem
         */
        void tick();

        /**
         * @brief Reads a precized number of bytes and place it into the buffer
         * @param size The number of bytes to be read
         * @warning Blocking function
         */
        void recv(unsigned short size);
    };
} // namespace client

#endif