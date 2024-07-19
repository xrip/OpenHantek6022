// SPDX-License-Identifier: GPL-2.0-or-later

#include "modelRP2040.h"
#include "hantekdsocontrol.h"
#include "hantekprotocol/controlStructs.h"
#include "usb/scopedevice.h"
#include <QDebug>
#include <QDir>
#include <QSettings>


#define VERBOSE 0

using namespace Hantek;

static ModelRP2040 modelInstance_rp2040;

static void initSpecifications( Dso::ControlSpecification &specification ) {
    // we drop 2K + 480 sample values due to unreliable start of stream
    // 20000 samples at 100kS/s = 200 ms gives enough to fill
    // the screen two times (for pre/post trigger) at 10ms/div = 100ms/screen
    // SAMPLESIZE defined in hantekdsocontrol.h
    // adapt accordingly in HantekDsoControl::convertRawDataToSamples()

    // HW gain, voltage steps in V/div (ranges 20,50,100,200,500,1000,2000,5000 mV)
    specification.gain = { { 10, 20e-3 }, { 10, 50e-3 }, { 10, 100e-3 }, { 5, 200e-3 },
                           { 2, 500e-3 }, { 1, 1.00 },   { 1, 2.00 },    { 1, 5.00 } };

    // Define the scaling between ADC sample values and real input voltage
    // Everything is scaled on the full screen height (8 divs)
    // The voltage/div setting:      20m   50m  100m  200m  500m    1V    2V    5V
    // Equivalent input voltage:   0.16V  0.4V  0.8V  1.6V    4V    8V   16V   40V
    // Theoretical gain setting:     x10   x10   x10   x5    x2     x1    x1    x1
    // mV / digit:                     4     4     4     8    20    40    40    40
    // The real input front end introduces a gain error
    // Input divider: 100/1009 = 1% too low display
    // Amplifier gain: x1 (ok), x2 (ok), x5.1 (2% too high), x10.1 (1% too high)
    // Overall resulting gain: x1 1% too low, x2 1% to low, x5 1% to high, x10 ok
    specification.voltageScale[ 0 ] = { 250, 250, 250, 126.25, 49.50, 24.75, 24.75, 24.75 };
    specification.voltageScale[ 1 ] = { 250, 250, 250, 126.25, 49.50, 24.75, 24.75, 24.75 };
    // Gain and offset can be corrected by individual config values from EEPROM or file

    // Possible raw sample rates with custom fw from https://github.com/Ho-Ro/Hantek6022API
    // 20k, 40k, 50k, 64k, 100k, 200k, 400k, 500k, 1M, 2M, 3M, 4M, 5M, 6M, 8M, 10M, 12M, 15M, 16M, 24M, 30M (, 48M)
    // 48M is unusable in 1 channel mode due to massive USB overrun
    // 24M, 30M and 48M are unusable in 2 channel mode
    // these unstable settings are disabled
    // Lower effective sample rates < 10 MS/s use oversampling to increase the SNR

    specification.samplerate.single.base = 100e3;
    specification.samplerate.single.max = 200e6;
    specification.samplerate.single.recordLengths = { UINT_MAX };
    specification.samplerate.multi.base = 100e3;
    specification.samplerate.multi.max = 100e3;
    specification.samplerate.multi.recordLengths = { UINT_MAX };

    specification.fixedSampleRates = {
        // samplerate, sampleId, downsampling
        { 1e3, 101, 10 },
        { 2e3, 102, 10 },
        { 5e3, 105, 10 },
        { 10e3, 101, 1 },
        { 20e3, 102, 1 },
        { 50e3, 105, 1 },
        { 100e3, 110, 1 },
        { 200e3, 120, 1 }
    };

    specification.couplings = { Dso::Coupling::DC};
    specification.triggerModes = {
        Dso::TriggerMode::AUTO,
        Dso::TriggerMode::NORMAL,
        Dso::TriggerMode::SINGLE,
        Dso::TriggerMode::ROLL,
    };

    specification.fixedUSBinLength = 64;

    specification.calfreqSteps = { 30,   40,   50,   60,   80,   100,  120,  160,  200,  250,  300,  400,  440,
                                   500,  600,  660,  800,  1000, 1200, 1600, 2000, 2500, 3300, 4000, 5000, 6000,
                                   8000, 10e3, 12e3, 16e3, 20e3, 25e3, 30e3, 40e3, 50e3, 60e3, 80e3, 100e3 };
    specification.hasCalibrationEEPROM = false;
}

static void applyRequirements_( HantekDsoControl *dsoControl ) {
    dsoControl->addCommand( new ControlSetGain_CH1() );    // 0xE0
    dsoControl->addCommand( new ControlSetGain_CH2() );    // 0xE1
    dsoControl->addCommand( new ControlSetSamplerate() );  // 0xE2
    dsoControl->addCommand( new ControlStartSampling() );  // 0xE3
    dsoControl->addCommand( new ControlSetNumChannels() ); // 0xE4
    dsoControl->addCommand( new ControlSetCoupling() );    // 0xE5 (no effect w/o AC/DC HW mod)
    dsoControl->addCommand( new ControlSetCalFreq() );     // 0xE6
}

///                  VID/PID active  VID/PID no FW         Scope name
//                  |------------|  |------------|         |------|
ModelRP2040::ModelRP2040()
    : DSOModel( ID, 0x04b5, 0x2040, 0x04b5, 0x2040, 0, "", "RP2040", Dso::ControlSpecification( 2 ) ) {
    initSpecifications( specification );
}

void ModelRP2040::applyRequirements( HantekDsoControl *dsoControl ) const { applyRequirements_( dsoControl ); }


