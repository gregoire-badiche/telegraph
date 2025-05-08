#ifndef TELEGRAPH_H
#define TELEGRAPH_H

#include <Arduino.h>
#include <assert.h>
#include <limits.h>

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

namespace telegraph
{
    /**
     * Implementation of a queue, using a fixed size array for better efficiency
     */
    class StackBuffer
    {
    protected:
        unsigned short start;
        unsigned short stop;
        byte data[BUFFER_MAX_SIZE];
        char data_str[BUFFER_MAX_SIZE + 1];

    public:
        StackBuffer()
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

        char *get_str()
        {
            for (unsigned short i = 0; i < size(); i++)
            {
                data_str[i] = data[index(i)];
            }

            start = stop;

            data[size()] = 0;

            return data_str;
        }

        unsigned short index(unsigned short i)
        {
            return i & (BUFFER_MAX_SIZE - 1);
        }
    };

    class Channel
    {
    protected:
        int pin;
        StackBuffer buffer;
        unsigned int freq;
        unsigned int delta_us;
        unsigned long _time;
        void reset_channel();
        bool _activated;
        bool _available;
        byte _curr_val;
        unsigned short _n_bits;
        bool _previous_reading;
        bool _mid_activated;

    public:
        Channel() {};
        Channel(int pin, unsigned int baud_rate);
        bool activated();
        bool available();
        unsigned short buff_size();
        char *read_buff();
    };

    class TransmitChannel : public Channel
    {
    private:
        void begin_transmission();
        void end_transmission();
        void write(byte data);
        void transmit_async();

    public:
        TransmitChannel() {};
        TransmitChannel(int pin, unsigned int baud_rate);
        void begin();

        /**
         * async
         */
        void send(byte *data, unsigned short size);

        /**
         * sync
         */
        void tell(byte *data, unsigned short size);
        void tell(byte data);
        void tick();
    };

    class RecieveChannel : public Channel
    {
    private:
        void wait_activation(unsigned int min_delay_us);
        void recieve_async();

    public:
        RecieveChannel() {};
        RecieveChannel(int pin, unsigned int baud_rate);
        void begin();
        byte read();
        byte peek();
        void await();
        void recieve(unsigned short size);
        void tick();
        bool available();
    };

    class Telegraph
    {
    private:
        TransmitChannel txs_arr[MAX_LISTENERS];
        RecieveChannel rxs_arr[MAX_LISTENERS];

    public:
        unsigned short n_listeners = 0;
        unsigned short n_talkers = 0;
        /**
         * @brief Constructor for a Telgraph instance
         */
        Telegraph() {};

        TransmitChannel &txs(uint8_t n);
        RecieveChannel &rxs(uint8_t n);

        // First instance of rx and tx
        TransmitChannel &tx();
        RecieveChannel &rx();

        /**
         * @brief Listen to a specific pins, and register it as a module
         * @return The ID of the connection (used to identify the buffer, the pin...)
         */
        uint8_t listen(int rx_pin, unsigned int baud_rate);

        /**
         *
         */
        uint8_t talk(int tx_pin, unsigned int baud_rate);

        /**
         * @brief Tell the module that the master is ready, and sets the baud rate
         * @param frequency Baud rate
         */
        void begin();

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
};

#endif
