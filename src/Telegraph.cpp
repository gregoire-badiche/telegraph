#include <Arduino.h>
#include <limits.h>
#include "Telegraph.h"

#define check_id assert(id > n_listeners || id <= 0)

namespace
{
    unsigned long delta_ulong(unsigned long a, unsigned long b)
    {
        // if (a < b)
        // {
        //     return a + ULONG_MAX - b;
        // }
        // if (a == b)
        // {
        //     return 0;
        // }
        return a - b;
    }

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
        Serial.println(n_listeners);
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
        for (unsigned int i = 0; i < size; i++)
        {
            begin_transmission();
            transmit_byte(data[i]);
            end_transmission();
        }
    }

    int Telegraph::buff_size(unsigned short id)
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
        while (delta_ulong(micros(), t) < MIN_AVAILABLE_DELTA)
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
                if (delta_ulong(micros(), times[i]) < MIN_AVAILABLE_DELTA)
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

    void Telegraph::tick()
    {
        for (unsigned short i = 0; i < n_listeners; i++)
        {
            if (channels[i].available)
            {
                if (channels[i].activated == true)
                {
                    if (channels[i].n_bits_read >= 8)
                    {
                        if (buffers[i].size() < BUFFER_MAX_SIZE)
                            buffers[i].push(channels[i].val);
                        reset_channel(i);
                        continue;
                    }

                    unsigned long t = micros();
                    if (delta_ulong(t, channels[i].time) >= delta_us)
                    {
                        channels[i].val <<= 1;
                        channels[i].val += (short)digitalRead(rx_pins[i]);
                        channels[i].time += delta_us;
                        channels[i].n_bits_read++;
                    }
                }
                else
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
                    }
                    else
                    {
                        if (channels[i].previous_reading == HIGH && delta_ulong(micros(), channels[i].time) > delta_us / 2)
                        {
                            channels[i].mid_activated = true;
                            channels[i].time = micros();
                        }
                        if (channels[i].previous_reading == LOW && channels[i].mid_activated == true && delta_ulong(micros(), channels[i].time) > delta_us / 2)
                        {
                            channels[i].activated = true;
                            channels[i].time += delta_us / 2;
                        }

                        channels[i].previous_reading = LOW;
                    }
                }
            }
            else
            {
                if (digitalRead(rx_pins[i]) == LOW)
                {
                    channels[i].time = 0;
                    continue;
                }

                if (channels[i].time == 0)
                {
                    channels[i].time = micros();
                }
                else if (delta_ulong(micros(), channels[i].time) > MIN_AVAILABLE_DELTA)
                {
                    channels[i].available = true;
                    channels[i].time = 0;
                }
            }
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
            digitalWrite(tx_pin, data & 0x80);
            data <<= 1;
            precise_delay(delta_us + t - micros());
            t += delta_us;
        }
    }

    void Telegraph::reset_channel(unsigned short id)
    {
        check_id;
        channels[id].activated = 0;
        channels[id].time = 0;
        channels[id].mid_activated = 0;
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
        for (unsigned short i = 0; i < size; i++)
        {
            begin_transmission();
            transmit_byte(data[i]);
            end_transmission();
        }
    }

    int Telegraph::buff_size()
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
        while (delta_ulong(micros(), t) < MIN_AVAILABLE_DELTA)
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
            digitalWrite(tx_pin, data & 0x80);
            data <<= 1;
            precise_delay(delta_us + t - micros());
            t += delta_us;
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

    void Telegraph::tick()
    {
        if (channel.available)
        {
            if (channel.activated == true)
            {
                if (channel.n_bits_read >= 8)
                {
                    if (buffer.size() < BUFFER_MAX_SIZE)
                        buffer.push(channel.val);
                    reset_channel();
                    return;
                }

                if (delta_ulong(micros(), channel.time) >= delta_us)
                {
                    channel.val <<= 1;
                    channel.val += (short)digitalRead(rx_pin);
                    channel.time += delta_us;
                    channel.n_bits_read++;
                }
            }
            else
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
                }
                else
                {
                    if (channel.previous_reading == HIGH && delta_ulong(micros(), channel.time) > delta_us / 2)
                    {
                        channel.mid_activated = true;
                        channel.time = micros();
                    }
                    if (channel.previous_reading == LOW && channel.mid_activated == true && delta_ulong(micros(), channel.time) > delta_us / 2)
                    {
                        channel.activated = true;
                        channel.time += delta_us / 2;
                    }

                    channel.previous_reading = LOW;
                }
            }
        }
        else
        {
            if (digitalRead(rx_pin) == LOW)
            {
                channel.time = 0;
                return;
            }

            if (channel.time == 0)
            {
                channel.time = micros();
            }
            else if (delta_ulong(micros(), channel.time) > MIN_AVAILABLE_DELTA)
            {
                channel.available = true;
                channel.time = 0;
            }
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

            precise_delay(delta_us);

            unsigned long t = micros();
            for (int j = 0; j < 8; j++)
            {
                val <<= 1;
                val += (short)digitalRead(rx_pin);
                precise_delay(t + delta_us - micros());
                t += delta_us;
            }

            buffer.push(val);
        }
    }
} // namespace client