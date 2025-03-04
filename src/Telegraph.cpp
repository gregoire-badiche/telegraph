#include <Arduino.h>
#include <limits.h>
#include "Telegraph.h"

#define check_id assert(id > n_listeners || id <= 0)

namespace telegraph
{
    Channel::Channel(int pin, unsigned int baud_rate) : pin(pin)
    {
        freq = baud_rate;
        delta_us = 1000000 / freq;
        buffer = StackBuffer();
        delta_us = 0;
        reset_channel();
    }

    void Channel::reset_channel()
    {
        _time = 0;
        _activated = 0;
        _curr_val = 0;
        _n_bits = 0;
        _previous_reading = 0;
        _mid_activated = 0;
    }

    bool Channel::activated()
    {
        return _activated;
    }

    bool Channel::available()
    {
        return _available;
    }

    unsigned short Channel::buff_size()
    {
        return buffer.size();
    }

    TransmitChannel::TransmitChannel(int pin, unsigned int baud_rate) : Channel(pin, baud_rate) {}

    void TransmitChannel::begin_transmission()
    {
        digitalWrite(pin, HIGH);
        precise_delay(delta_us);
        digitalWrite(pin, LOW);
        precise_delay(delta_us);
    }

    void TransmitChannel::end_transmission()
    {
        digitalWrite(pin, HIGH);
    }

    void TransmitChannel::write(byte data)
    {
        unsigned long t = micros();
        for (int i = 0; i < 8; i++)
        {
            digitalWrite(pin, data & 0x01);
            data >>= 1;
            t += delta_us;
            precise_delay(t - micros());
        }
    }

    void TransmitChannel::transmit_async()
    {
        if (buff_size() == 0)
            return;

        // Activation sequence
        if (_time == 0)
        {
            _n_bits = 0x01;
            digitalWrite(pin, HIGH);
            _time = micros();
            _activated = false;
            return;
        }
        if (_activated == false)
        {
            if (delta_ulong(micros(), _time) >= delta_us)
            {
                digitalWrite(pin, LOW);
                _time += delta_us;
                _activated = true;
            }
            return;
        }

        // End of the message
        if (_n_bits == 0x00 && delta_ulong(micros(), _time) >= delta_us)
        {
            digitalWrite(pin, HIGH);
            buffer.pop();
            _activated = false;
            _time = 0;
            return;
        }

        // Tranmission of the message
        if (delta_ulong(micros(), _time) >= delta_us)
        {
            digitalWrite(pin, (buffer.peek() & _n_bits) || 0);
            _n_bits <<= 1;
            _time += delta_us;
            return;
        }
    }

    void TransmitChannel::begin()
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }

    void TransmitChannel::tell(byte *data, unsigned short size)
    {
        for (unsigned short i = 0; i < size; i++)
        {
            begin_transmission();
            write(data[i]);
            end_transmission();
        }
    }

    void TransmitChannel::send(byte *data, unsigned short size)
    {
        assert(size + buff_size() < BUFFER_MAX_SIZE);

        for (unsigned short i = 0; i < size; i++)
        {
            buffer.push(data[i]);
        }
    }

    void TransmitChannel::tick()
    {
        transmit_async();
    }

    RecieveChannel::RecieveChannel(int pin, unsigned int baud_rate) : Channel(pin, baud_rate) {}

    void RecieveChannel::wait_activation(unsigned int min_delay_us)
    {
        bool activated = false;
        bool previous_reading = LOW;
        bool p = false;
        unsigned int time = 0;

        while (!activated)
        {
            bool r = digitalRead(pin);
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

    void RecieveChannel::recieve_async()
    {
        // If the channel is unavailable
        if (!_available)
        {
            if (digitalRead(pin) == LOW || _time == 0)
            {
                _time = micros();
            }
            else if (delta_ulong(micros(), _time) >= MIN_AVAILABLE_DELTA)
            {
                _available = true;
                _time = 0;
            }

            return;
        }

        // If the channel is available but isn't activated yet
        if (!_activated)
        {
            bool r = digitalRead(pin);
            if (r == HIGH)
            {
                if (_previous_reading == LOW)
                {
                    _time = micros();
                }
                _mid_activated = false;
                _previous_reading = HIGH;

                return;
            }

            if (delta_ulong(micros(), _time) < delta_us / 2)
                return;

            if (_previous_reading == HIGH)
            {
                _mid_activated = true;
                _time = micros();
            }

            if (_previous_reading == LOW && _mid_activated == true)
            {
                _activated = true;
                _time += delta_us / 2;
            }

            _previous_reading = LOW;
            return;
        }

        // If the channel is available, activated, and we need to read
        if (delta_ulong(micros(), _time) >= delta_us)
        {
            _curr_val >>= 1;
            _curr_val += (char)digitalRead(pin) << 7;
            _time += delta_us;
            _n_bits++;
        }

        // If the channel is available and activated, but it's the end of the byte
        if (_n_bits >= 8)
        {
            if (buffer.size() < BUFFER_MAX_SIZE)
                buffer.push(_curr_val);
            reset_channel();
        }
    }

    void RecieveChannel::recieve(unsigned short size)
    {
        assert(size < BUFFER_MAX_SIZE);

        if (!_available)
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
                val += (char)digitalRead(pin) << 7;
            }

            buffer.push(val);
        }
    }

    void RecieveChannel::begin()
    {
        pinMode(pin, INPUT);
    }

    byte RecieveChannel::read()
    {
        return buffer.pop();
    }

    byte RecieveChannel::peek()
    {
        return buffer.peek();
    }

    void RecieveChannel::await()
    {
        unsigned long t = micros();
        while (delta_ulong(micros(), t) <= MIN_AVAILABLE_DELTA)
        {
            if (digitalRead(pin) == LOW)
            {
                t = micros();
            }
        }
        _available = true;
    }

    void RecieveChannel::tick()
    {
        recieve_async();
    }

    uint8_t Telegraph::listen(int rx_pin, unsigned int baud_rate)
    {
        rxs_arr[n_listeners] = RecieveChannel(rx_pin, baud_rate);
        return n_listeners++;
    }

    uint8_t Telegraph::talk(int rx_pin, unsigned int baud_rate)
    {
        txs_arr[n_talkers] = TransmitChannel(rx_pin, baud_rate);
        return n_talkers++;
    }

    void Telegraph::begin()
    {
        for (uint8_t i = 0; i < n_listeners; i++)
        {
            rxs_arr[i].begin();
        }

        for (uint8_t i = 0; i < n_talkers; i++)
        {
            txs_arr[i].begin();
        }
    }

    void Telegraph::await_all()
    {
        bool ready = false;

        while (!ready)
        {
            ready = true;
            for (uint8_t i = 0; i < n_listeners; i++)
            {
                if (!rxs_arr[i].available())
                {
                    ready = false;
                }
            }
        }
    }

    void Telegraph::tick()
    {
        for (uint8_t i = 0; i < n_listeners; i++)
        {
            rxs_arr[i].tick();
        }

        for (uint8_t i = 0; i < n_talkers; i++)
        {
            txs_arr[i].tick();
        }
    }

    TransmitChannel &Telegraph::txs(uint8_t n)
    {
        assert(n < n_talkers);
        return txs_arr[n];
    }

    RecieveChannel &Telegraph::rxs(uint8_t n)
    {
        assert(n < n_listeners);
        return rxs_arr[n];
    }

    TransmitChannel &Telegraph::tx()
    {
        assert(0 < n_listeners);
        return txs_arr[0];
    }

    RecieveChannel &Telegraph::rx()
    {
        assert(0 < n_listeners);
        return rxs_arr[0];
    }
}
