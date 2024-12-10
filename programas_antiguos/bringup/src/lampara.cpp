
/*
Copyright (c) 2012-2017 Ben Croston <ben@croston.org>.
Copyright (c) 2019-2023, NVIDIA CORPORATION.
Copyright (c) 2019-2023, Jueon Park(pjueon) <bluegbgb@gmail.com>.
Copyright (c) 2021-2023, Adam Rasburn <blackforestcheesecake@protonmail.ch>.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "JetsonGPIO.h"
#include "private/ExceptionHandling.h"
#include "private/GPIOEvent.h"
#include "private/GPIOPinData.h"
#include "private/MainModule.h"
#include "private/Model.h"
#include "private/ModelUtility.h"
#include "private/PythonFunctions.h"
#include "private/SysfsRoot.h"

// public APIs
namespace GPIO
{
    LazyString model{[]() { return global().model(); }};
    LazyString JETSON_INFO{[]() { return global().JETSON_INFO(); }};

    void setwarnings(bool state) { global()._gpio_warnings = state; }

    void setmode(NumberingModes mode)
    {
        try
        {
            // check if mode is valid
            if (mode == NumberingModes::None)
                throw std::runtime_error("Pin numbering mode must be BOARD, BCM, TEGRA_SOC or CVM");

            // check if a different mode has been set
            bool already_set = global()._gpio_mode != NumberingModes::None;
            bool same_mode = mode == global()._gpio_mode;

            if (already_set && !same_mode)
            {
                throw std::runtime_error("A different mode has already been set!");
            }
            else if (already_set && same_mode)
            {
                return; // do nothing
            }
            else // not set yet
            {
                global()._channel_data = global()._channel_data_by_mode.at(mode);
                global()._gpio_mode = mode;
            }
        }
        catch (std::exception& e)
        {
            throw _error(e, "setmode()");
        }
    }

    NumberingModes getmode() { return global()._gpio_mode; }

    void setup(const std::string& channel, Directions direction, int initial)
    {
        try
        {
            ChannelInfo ch_info = global()._channel_to_info(channel, true);

            if (global()._gpio_warnings)
            {
                Directions sysfs_cfg = global()._sysfs_channel_configuration(ch_info);
                Directions app_cfg = global()._app_channel_configuration(ch_info);

                if (app_cfg == UNKNOWN && sysfs_cfg != UNKNOWN)
                {
                    std::cerr
                        << "[WARNING] This channel is already in use, continuing anyway. Use setwarnings(false) to "
                           "disable warnings. channel: "
                        << channel << std::endl;
                }
            }

            if (is_in(ch_info.channel, global()._channel_configuration))
                global()._cleanup_one(ch_info);

            if (direction == OUT)
            {
                global()._setup_single_out(ch_info, initial);
            }
            else if (direction == IN)
            {
                if (!is_None(initial))
                    throw std::runtime_error("initial parameter is not valid for inputs");
                global()._setup_single_in(ch_info);
            }
            else
                throw std::runtime_error("GPIO direction must be IN or OUT");
        }
        catch (std::exception& e)
        {
            throw _error(e, "setup()");
        }
    }

    void setup(int channel, Directions direction, int initial) { setup(std::to_string(channel), direction, initial); }

    void setup(const std::vector<std::string>& channels, Directions direction, int initial)
    {
        for (const auto& channel : channels)
            setup(channel, direction, initial);
    }

    void setup(const std::vector<int>& channels, Directions direction, int initial)
    {
        for (const auto& channel : channels)
            setup(channel, direction, initial);
    }    

    void setup(const std::initializer_list<int>& channels, Directions direction, int initial)
    {
        setup(std::vector<int>(channels), direction, initial);
    }

    void setup(const std::vector<std::string>& channels, Directions direction, const std::vector<int>& initials)
    {
        if (direction == Directions::OUT && channels.size() != initials.size())
            throw std::runtime_error(format("Number of values (%d) != number of channels (%d)", initials.size(), channels.size()));

        for (std::size_t i = 0; i < channels.size(); i++)
            setup(channels[i], direction, initials[i]);    
    }

    void setup(const std::vector<int>& channels, Directions direction, const std::vector<int>& initials)
    {
        std::vector<std::string> _channels(channels.size());
        std::transform(channels.begin(), channels.end(), _channels.begin(),
                       [](int value) { return std::to_string(value); });

        setup(_channels, direction, initials);
    }

    void setup(const std::initializer_list<int>& channels, Directions direction, const std::vector<int>& initials)
    {
        setup(std::vector<int>(channels), direction, initials);
    }


    // clean all channels if no channel param provided
    void cleanup()
    {
        try
        {
            global()._warn_if_no_channel_to_cleanup();
            global()._cleanup_all();
        }
        catch (std::exception& e)
        {
            throw _error(e, "cleanup()");
        }
    }

    void cleanup(const std::vector<std::string>& channels)
    {
        try
        {
            global()._warn_if_no_channel_to_cleanup();

            auto ch_infos = global()._channels_to_infos(channels);
            for (auto&& ch_info : ch_infos)
            {
                if (is_in(ch_info.channel, global()._channel_configuration))
                {
                    global()._cleanup_one(ch_info);
                }
            }
        }
        catch (std::exception& e)
        {
            throw _error(e, "cleanup()");
        }
    }

    void cleanup(const std::vector<int>& channels)
    {
        std::vector<std::string> _channels(channels.size());
        std::transform(channels.begin(), channels.end(), _channels.begin(),
                       [](int value) { return std::to_string(value); });
        cleanup(_channels);
    }

    void cleanup(const std::string& channel) { cleanup(std::vector<std::string>{channel}); }

    void cleanup(int channel)
    {
        std::string str_channel = std::to_string(channel);
        cleanup(str_channel);
    }

    void cleanup(const std::initializer_list<int>& channels) { cleanup(std::vector<int>(channels)); }

    int input(const std::string& channel)
    {
        try
        {
            ChannelInfo ch_info = global()._channel_to_info(channel, true);

            Directions app_cfg = global()._app_channel_configuration(ch_info);

            if (app_cfg != IN && app_cfg != OUT)
                throw std::runtime_error("You must setup() the GPIO channel first");

            ch_info.f_value->seekg(0, std::ios::beg);
            int value_read{};
            *ch_info.f_value >> value_read;
            return value_read;
        }
        catch (std::exception& e)
        {
            throw _error(e, "input()");
        }
    }

    int input(int channel) { return input(std::to_string(channel)); }

    /* Function used to set a value to a channel.
       Values must be either HIGH or LOW */
    void output(const std::string& channel, int value)
    {
        try
        {
            ChannelInfo ch_info = global()._channel_to_info(channel, true);
            // check that the channel has been set as output
            if (global()._app_channel_configuration(ch_info) != OUT)
                throw std::runtime_error("The GPIO channel has not been set up as an OUTPUT");
            global()._output_one(ch_info, value);
        }
        catch (std::exception& e)
        {
            throw _error(e, "output()");
        }
    }

    void output(int channel, int value) { output(std::to_string(channel), value); }

    void output(const std::vector<std::string>& channels, int value)
    {
        output(channels, std::vector<int>(channels.size(), value));
    }

    void output(const std::initializer_list<int>& channels, int value)
    {
        output(std::vector<int>(channels), std::vector<int>(channels.size(), value));
    }

    void output(const std::vector<int>& channels, int value)
    {
        output(channels, std::vector<int>(channels.size(), value));
    }

    void output(const std::vector<std::string>& channels, const std::vector<int>& values)
    {
        if (channels.size() != values.size())
            throw std::runtime_error(format("Number of values (%d) != number of channels (%d)", values.size(), channels.size()));

        for (std::size_t i = 0; i < channels.size(); i++)
            output(channels[i], values[i]);
    }

    void output(const std::initializer_list<int>& channels, const std::vector<int>& values)
    {
        output(std::vector<int>(channels), values);
    }

    void output(const std::vector<int>& channels, const std::vector<int>& values)
    {
        std::vector<std::string> _channels(channels.size());
        std::transform(channels.begin(), channels.end(), _channels.begin(), [](int x){ return std::to_string(x); });
        output(_channels, values);
    }

    Directions gpio_function(const std::string& channel)
    {
        try
        {
            ChannelInfo ch_info = global()._channel_to_info(channel);
            return global()._sysfs_channel_configuration(ch_info);
        }
        catch (std::exception& e)
        {
            throw _error(e, "gpio_function()");
        }
    }

    Directions gpio_function(int channel) { return gpio_function(std::to_string(channel)); }

    //=============================== Events =================================

   
} // namespace GPIO



//metodo de numeración de pines (en este caso numérico BCM)
GPIO::setmode(GPIO::BCM);
const int pin1=1;
const int pin2=2;
const int pin3=3;
//pin 1 es de salida con valor inicial LOW
GPIO::setup(pin1, GPIO::OUT, GPIO::LOW);
GPIO::setup(pin2, GPIO::OUT, GPIO::LOW);
GPIO::setup(pin3, GPIO::OUT, GPIO::LOW);

static bool end_this_program = false;
bool inicio=true;
int main(){

while(!end_this_program){
    //se enciende cada luz por separado y luego se encienden todas
    if(inicio){
        GPIO::output(pin1, GPIO::HIGH);
        delayMs(200);
        GPIO::output(pin2, GPIO::HIGH);
        delayMs(200);
         GPIO::output(pin3, GPIO::HIGH);
        delayMs(200);
        inicio=false;

    }
    else{
        GPIO::output(pin1, GPIO::HIGH);
        GPIO::output(pin2, GPIO::HIGH);
        GPIO::output(pin3, GPIO::HIGH);
    }
}

//devuelve a estado inicial todos los pines
GPIO::cleanup();
return 0;

}



