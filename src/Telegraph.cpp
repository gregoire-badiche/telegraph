#include <Arduino.h>
#include <limits.h>
#include "Telegraph.h"

#define check_id assert(id > n_listeners || id <= 0)

namespace
{
    /**
     * @brief Compute a delta between two ulong, taking into account the possible overflow
     */
    unsigned long delta_ulong(unsigned long a, unsigned long b)
    {
        if (a < b)
        {
            return a + ULONG_MAX - b;
        }
        if (a == b)
        {
            return 0;
        }
        return a - b;
    }

    /**
     * @brief Long delay precise to the microseconds, can take any value (while delayMicroseconds() is limited in size)
     * @param us Delay in microseconds
     */
    void precise_delay(unsigned int us)
    {
        unsigned int m = us / 1000;
        unsigned int u = us - m * 1000;
        delay(m);
        delayMicroseconds(u);
    }
}

namespace master
{
    Telegraph::Telegraph(int tx_pin) : tx_pin(tx_pin)
    {
        for (int i = 0; i < MAX_LISTENERS; i++)
        {
            buffers[i] = list();
            reset_channel(i);
            channels[i].available = false;
            channels[i].id = i;
        }

        transmit.time = 0;
        transmit.u = false;
        transmit.bit_n = 0x00;
        transmit.data = list();
    }

    short Telegraph::listen(int rx_pin)
    {
        pinMode(rx_pin, INPUT);
        rx_pins[n_listeners] = rx_pin;
        return n_listeners++;
    }

    void Telegraph::begin(unsigned int frequency)
    {
        pinMode(tx_pin, OUTPUT);
        freq = frequency;
        delta_us = 1000000 / frequency;
        digitalWrite(tx_pin, HIGH);
    }

    byte Telegraph::read(unsigned short id)
    {
        check_id;
        return buffers[id].pop();
    }

    byte Telegraph::peek(unsigned short id)
    {
        check_id;
        return buffers[id].peek();
    }

    void Telegraph::write(byte *data, unsigned int size)
    {
        // Empty the transmit_async buffer
        while (!transmit.data.size())
        {
            tick();
        }
        
        for (unsigned int i = 0; i < size; i++)
        {
            begin_transmission();
            transmit_byte(data[i]);
            end_transmission();
        }
    }

    unsigned short Telegraph::buff_size(unsigned short id)
    {
        if (id > n_listeners || id < 0)
            return 0;
        return buffers[id].size();
    }

    bool Telegraph::available(unsigned short id)
    {
        check_id;
        return channels[id].available;
    }

    void Telegraph::await(unsigned short id)
    {
        check_id;

        unsigned long t = micros();
        while (delta_ulong(micros(), t) <= MIN_AVAILABLE_DELTA)
        {
            if (digitalRead(rx_pins[id]) == LOW)
            {
                t = micros();
            }
        }
        channels[id].available = true;
    }

    void Telegraph::await_all()
    {
        unsigned short s = n_listeners;

        unsigned long times[MAX_LISTENERS];

        for (unsigned short i = 0; i < n_listeners; i++)
        {
            times[i] = micros();
        }

        while (s != 0)
        {
            s = n_listeners;
            for (unsigned short i = 0; i < n_listeners; i++)
            {
                if (channels[i].available)
                {
                    s--;
                    continue;
                }
                if (delta_ulong(micros(), times[i]) <= MIN_AVAILABLE_DELTA)
                {
                    if (digitalRead(rx_pins[i]) == LOW)
                    {
                        times[i] = micros();
                    }
                }
                else
                {
                    s--;
                    channels[i].available = true;
                }
            }
        }
    }

    void Telegraph::recieve_async()
    {
        for (unsigned short i = 0; i < n_listeners; i++)
        {
            // If the channel is unavailable
            if (!channels[i].available)
            {
                if (digitalRead(rx_pins[i]) == LOW || channels[i].time == 0)
                {
                    channels[i].time = micros();
                }
                else if (delta_ulong(micros(), channels[i].time) >= MIN_AVAILABLE_DELTA)
                {
                    channels[i].available = true;
                    channels[i].time = 0;
                }

                return;
            }

            // If the channel is available but isn't activated yet
            if (!channels[i].activated)
            {
                bool r = digitalRead(rx_pins[i]);
                if (r == HIGH)
                {
                    if (channels[i].previous_reading == LOW)
                    {
                        channels[i].time = micros();
                    }
                    channels[i].mid_activated = false;
                    channels[i].previous_reading = HIGH;

                    return;
                }

                if (delta_ulong(micros(), channels[i].time) < delta_us / 2)
                    return;

                if (channels[i].previous_reading == HIGH)
                {
                    channels[i].mid_activated = true;
                    channels[i].time = micros();
                }

                if (channels[i].previous_reading == LOW && channels[i].mid_activated == true)
                {
                    channels[i].activated = true;
                    channels[i].time += delta_us / 2;
                }

                channels[i].previous_reading = LOW;
                return;
            }

            // If the channel is available, activated, and we need to read
            if (delta_ulong(micros(), channels[i].time) >= delta_us)
            {
                channels[i].val >>= 1;
                channels[i].val += (char)digitalRead(rx_pins[i]) << 7;
                channels[i].time += delta_us;
                channels[i].n_bits_read++;
            }

            // If the channel is available and activated, but it's the end of the byte
            if (channels[i].n_bits_read >= 8)
            {
                if (buffers[i].size() < BUFFER_MAX_SIZE)
                    buffers[i].push(channels[i].val);
                reset_channel(i);
            }
        }
    }

    void Telegraph::transmit_async()
    {
        if (transmit.data.size() == 0)
            return;

        // Activation sequence
        if (transmit.time == 0)
        {
            transmit.bit_n = 0x01;
            digitalWrite(tx_pin, HIGH);
            transmit.time = micros();
            transmit.u = false;
            return;
        }
        if (transmit.u == false)
        {
            if (delta_ulong(micros(), transmit.time) >= delta_us)
            {
                digitalWrite(tx_pin, LOW);
                transmit.time += delta_us;
                transmit.u = true;
            }
            return;
        }

        // End of the message
        if (transmit.bit_n == 0x00 && delta_ulong(micros(), transmit.time) >= delta_us)
        {
            digitalWrite(tx_pin, HIGH);
            transmit.data.pop();
            transmit.u = false;
            transmit.time = 0;
            return;
        }

        // Tranmission of the message
        if (delta_ulong(micros(), transmit.time) >= delta_us)
        {
            digitalWrite(tx_pin, (transmit.data.peek() & transmit.bit_n) || 0);
            transmit.bit_n <<= 1;
            transmit.time += delta_us;
            return;
        }
    }

    void Telegraph::tick()
    {
        recieve_async();
        transmit_async();
    }

    void Telegraph::send(byte *data, unsigned short size)
    {
        assert(size + transmit.data.size() < BUFFER_MAX_SIZE);

        for (unsigned short i = 0; i < size; i++)
        {
            transmit.data.push(data[i]);
        }
    }

    void Telegraph::begin_transmission()
    {
        digitalWrite(tx_pin, HIGH);
        precise_delay(delta_us);
        digitalWrite(tx_pin, LOW);
        precise_delay(delta_us);
    }

    void Telegraph::end_transmission()
    {
        digitalWrite(tx_pin, HIGH);
    }

    void Telegraph::transmit_byte(byte data)
    {
        unsigned long t = micros();
        for (int i = 0; i < 8; i++)
        {
            digitalWrite(tx_pin, data & 0x01);
            data >>= 1;
            t += delta_us;
            precise_delay(t - micros());
        }
    }

    void Telegraph::reset_channel(unsigned short id)
    {
        check_id;
        channels[id].activated = false;
        channels[id].time = 0;
        channels[id].mid_activated = false;
        channels[id].val = 0x00;
        channels[id].previous_reading = LOW;
        channels[id].n_bits_read = 0;
    }
} // namespace master

namespace client
{
    Telegraph::Telegraph(int rx_pin, int tx_pin) : rx_pin(rx_pin), tx_pin(tx_pin)
    {
        buffer = list();
        reset_channel();
        channel.available = false;
        channel.id = 0;

        transmit.time = 0;
        transmit.u = false;
        transmit.bit_n = 0x00;
        transmit.data = list();
    }

    void Telegraph::begin(unsigned int frequency)
    {
        pinMode(tx_pin, OUTPUT);
        pinMode(rx_pin, INPUT);
        freq = frequency;
        delta_us = 1000000 / frequency;
        digitalWrite(tx_pin, HIGH);
    }

    byte Telegraph::read()
    {
        return buffer.pop();
    }

    byte Telegraph::peek()
    {
        return buffer.peek();
    }

    void Telegraph::write(byte *data, unsigned int size)
    {
        // Empty the transmit_async buffer
        while (!transmit.data.size())
        {
            tick();
        }

        for (unsigned short i = 0; i < size; i++)
        {
            begin_transmission();
            transmit_byte(data[i]);
            end_transmission();
        }
    }

    unsigned short Telegraph::buff_size()
    {
        return buffer.size();
    }

    bool Telegraph::available()
    {
        return channel.available;
    }

    void Telegraph::await()
    {
        unsigned long t = micros();
        while (delta_ulong(micros(), t) <= MIN_AVAILABLE_DELTA)
        {
            if (digitalRead(rx_pin) == LOW)
            {
                t = micros();
            }
        }
        channel.available = true;
    }

    void Telegraph::begin_transmission()
    {
        digitalWrite(tx_pin, HIGH);
        precise_delay(delta_us);
        digitalWrite(tx_pin, LOW);
        precise_delay(delta_us);
    }

    void Telegraph::end_transmission()
    {
        digitalWrite(tx_pin, HIGH);
    }

    void Telegraph::transmit_byte(byte data)
    {
        unsigned long t = micros();
        for (int i = 0; i < 8; i++)
        {
            digitalWrite(tx_pin, data & 0x01);
            data >>= 1;
            t += delta_us;
            precise_delay(t - micros());
        }
    }

    void Telegraph::wait_activation(unsigned int min_delay_us)
    {
        bool activated = false;
        bool previous_reading = LOW;
        bool p = false;
        unsigned int time = 0;

        while (!activated)
        {
            bool r = digitalRead(rx_pin);
            if (r == HIGH)
            {
                if (previous_reading == LOW)
                {
                    time = micros();
                }
                p = false;
                previous_reading = HIGH;
            }
            else
            {
                if (previous_reading == HIGH && delta_ulong(micros(), time) >= min_delay_us)
                {
                    p = true;
                    time += delta_us;
                }
                if (previous_reading == LOW && p == true && delta_ulong(micros(), time) >= min_delay_us)
                {
                    activated = true;
                }

                previous_reading = LOW;
            }
        }
    }

    void Telegraph::reset_channel()
    {
        channel.activated = 0;
        channel.time = 0;
        channel.mid_activated = 0;
        channel.val = 0x00;
        channel.previous_reading = LOW;
        channel.n_bits_read = 0;
    }

    void Telegraph::recieve_async()
    {
        // If the channel is unavailable
        if (!channel.available)
        {
            if (digitalRead(rx_pin) == LOW || channel.time == 0)
            {
                channel.time = micros();
            }
            else if (delta_ulong(micros(), channel.time) >= MIN_AVAILABLE_DELTA)
            {
                channel.available = true;
                channel.time = 0;
            }

            return;
        }

        // If the channel is available but isn't activated yet
        if (!channel.activated)
        {
            bool r = digitalRead(rx_pin);
            if (r == HIGH)
            {
                if (channel.previous_reading == LOW)
                {
                    channel.time = micros();
                }
                channel.mid_activated = false;
                channel.previous_reading = HIGH;

                return;
            }

            if (delta_ulong(micros(), channel.time) < delta_us / 2)
                return;

            if (channel.previous_reading == HIGH)
            {
                channel.mid_activated = true;
                channel.time = micros();
            }

            if (channel.previous_reading == LOW && channel.mid_activated == true)
            {
                channel.activated = true;
                channel.time += delta_us / 2;
            }

            channel.previous_reading = LOW;
            return;
        }

        // If the channel is available, activated, and we need to read
        if (delta_ulong(micros(), channel.time) >= delta_us)
        {
            channel.val >>= 1;
            channel.val += (char)digitalRead(rx_pin) << 7;
            channel.time += delta_us;
            channel.n_bits_read++;
        }

        // If the channel is available and activated, but it's the end of the byte
        if (channel.n_bits_read >= 8)
        {
            if (buffer.size() < BUFFER_MAX_SIZE)
                buffer.push(channel.val);
            reset_channel();
        }
    }

    void Telegraph::transmit_async()
    {
        if (transmit.data.size() == 0)
            return;

        // Activation sequence
        if (transmit.time == 0)
        {
            transmit.bit_n = 0x01;
            digitalWrite(tx_pin, HIGH);
            transmit.time = micros();
            transmit.u = false;
            return;
        }
        if (transmit.u == false)
        {
            if (delta_ulong(micros(), transmit.time) >= delta_us)
            {
                digitalWrite(tx_pin, LOW);
                transmit.time += delta_us;
                transmit.u = true;
            }
            return;
        }

        // End of the message
        if (transmit.bit_n == 0x00 && delta_ulong(micros(), transmit.time) >= delta_us)
        {
            digitalWrite(tx_pin, HIGH);
            transmit.data.pop();
            transmit.u = false;
            transmit.time = 0;
            return;
        }

        // Tranmission of the message
        if (delta_ulong(micros(), transmit.time) >= delta_us)
        {
            digitalWrite(tx_pin, (transmit.data.peek() & transmit.bit_n) || 0);
            transmit.bit_n <<= 1;
            transmit.time += delta_us;
            return;
        }
    }

    void Telegraph::tick()
    {
        recieve_async();
        transmit_async();
    }

    void Telegraph::send(byte *data, unsigned short size)
    {
        assert(size + transmit.data.size() < BUFFER_MAX_SIZE);

        for (unsigned short i = 0; i < size; i++)
        {
            transmit.data.push(data[i]);
        }
    }

    void Telegraph::recv(unsigned short size)
    {
        assert(size < BUFFER_MAX_SIZE);

        if (!channel.available)
            await();

        buffer.clear();

        // Waits for the end of any currently incoming messages
        precise_delay(delta_us * 9);

        for (unsigned short i = 0; i < size; i++)
        {
            reset_channel();
            unsigned short val = 0x0;

            wait_activation(delta_us / 2);

            unsigned long t = micros();
            for (int j = 0; j < 8; j++)
            {
                t += delta_us;
                precise_delay(t - micros());
                val >>= 1;
                val += (char)digitalRead(rx_pin) << 7;
            }

            buffer.push(val);
        }
    }
} // namespace client