//
// EKS Otus HID controller script v1.0
// Copyright (C) 2012, Sean M. Pappalardo, Ilkka Tuohela
// but feel free to tweak this to your heart's content!
// For Mixxx version 1.11.x
//

// --- Compatibility shim for Mixxx 2.5's common-hid-packet-parser.js -------
// This script's output linking (controller.linkOutput()/EksOtus.outputCallback,
// unchanged since Mixxx 2.2 - verified against the 2.2.0 release source)
// depends on HIDController.getOutputField(m_group, m_name) being able to find
// an output field by its *mapped* Mixxx group/name (e.g. "deck","play"), not
// just its raw HID group/name ("hid","play").
//
// In Mixxx 2.2 this worked because getOutputField() did a live linear scan
// over every registered output field on every call, checking both
// field.mapped_group/mapped_name AND field.group/name - so it kept working
// no matter when a field got linked.
//
// In Mixxx 2.5 this was rewritten as an O(1) OutputFieldLookup Map for
// performance. registerOutputPacket() populates that Map from both the raw
// and mapped identities, but only once, at packet registration time - before
// any linkOutput() call has run, so mapped_group/mapped_name don't exist yet.
// The Map is never updated afterwards, so every lookup by mapped identity
// (linkOutput()'s own initial LED sync, and every call to outputCallback()
// on a subsequent control change) misses and logs
// "HIDController.setOutput - Unknown field: deck.X". This is a genuine
// engine regression, not a bug in this driver - restore the old fallback
// behavior here (and cache the result back into the Map, so this only
// costs a scan once per field).
if (typeof HIDController !== "undefined" &&
        HIDController.prototype.getOutputField &&
        !HIDController.prototype._eksOtusGetOutputFieldPatched) {
    var _eksOtusOriginalGetOutputField = HIDController.prototype.getOutputField;
    HIDController.prototype.getOutputField = function(m_group, m_name) {
        var field = _eksOtusOriginalGetOutputField.call(this, m_group, m_name);
        if (field !== undefined) {
            return field;
        }
        // Fall back to a Mixxx-2.2-style linear scan by mapped identity.
        for (var packet_name in this.OutputPackets) {
            var packet = this.OutputPackets[packet_name];
            for (var group_name in packet.groups) {
                var group = packet.groups[group_name];
                for (var field_name in group) {
                    var candidate = group[field_name];
                    if (candidate.type === "bitvector") {
                        for (var bit_id in candidate.value.bits) {
                            var bit = candidate.value.bits[bit_id];
                            if (bit.mapped_group === m_group && bit.mapped_name === m_name) {
                                this.OutputFieldLookup.set([m_group, m_name].toString(), bit);
                                return bit;
                            }
                        }
                        continue;
                    }
                    if (candidate.mapped_group === m_group && candidate.mapped_name === m_name) {
                        this.OutputFieldLookup.set([m_group, m_name].toString(), candidate);
                        return candidate;
                    }
                }
            }
        }
        return undefined;
    };
    HIDController.prototype._eksOtusGetOutputFieldPatched = true;
}
// ---------------------------------------------------------------------------

// EKS Otus HID interface specification
function EKSOtusController() {
    this.controller = new HIDController();

    // Initialized to firmware version by version response packet
    this.version_major = undefined;
    this.version_minor = undefined;
    this.controller.activeDeck = 1;

    this.controller.LEDColors = { off: 0x0, red: 0x0f, green: 0xf0, amber: 0xff };
    this.controller.deckOutputColors = { 1: "red", 2: "green", 3: "red", 4: "green"};

    // Static variables for HID specs
    this.wheelLEDCount = 60;
    this.buttonLEDCount = 22;
    this.sliderLEDCount = 20;

    this.registerInputPackets = function() {
        var packet = undefined;
        var name = undefined;
        var offset = 0;

        // The control report arrives as 00 35 ... on BOTH macOS and Windows, so
        // [0x00, 0x35] is the correct header everywhere. (Upstream main's [0x35]
        // is wrong on every platform.) Measured on Windows from a freshly
        // replugged device with nothing written to it:
        //     00 35 A4 55 E0 00 2F CF 45 00 1E 9D ...
        //     00 35 E0 57 3C 02 02 FE 45 00 1E 9D ...
        // 6612 reports in 40 s, wheel_position live at offset 2 as mapped below.
        packet = new HIDPacket("control", 0, undefined, [0x00, 0x35]);
        packet.addControl("hid","wheel_position",2,"H");
        packet.addControl("hid","wheel_speed",4,"h");
        packet.addControl("hid","timestamp",6,"I");
        packet.addControl("hid","slider_value",10,"H");
        packet.addControl("hid","slider_position",12,"H");
        // The four corner encoders. Offsets 14 and 17 were BOTH named
        // "rate_encoder": since a field's id is group.name they collided, so one
        // of the two top encoders was unreachable no matter what was bound to
        // it. Mapped to corners by turning each one in isolation:
        //   offset 17 = top-left      offset 14 = top-right
        //   offset 16 = bottom-left   offset 15 = bottom-right
        packet.addControl("hid","jog_ne",14,"B",undefined,true);
        packet.addControl("hid","jog_se",15,"B",undefined,true);
        packet.addControl("hid","jog_sw",16,"B",undefined,true);
        packet.addControl("hid","jog_nw",17,"B",undefined,true);
        packet.addControl("hid","gain_1",18,"H");
        packet.addControl("hid","gain_2",20,"H");
        packet.addControl("hid","eq_high_1",22,"H");
        packet.addControl("hid","eq_high_2",24,"H");
        packet.addControl("hid","eq_mid_1",26,"H");
        packet.addControl("hid","eq_mid_2",28,"H");
        packet.addControl("hid","eq_low_1",30,"H");
        packet.addControl("hid","eq_low_2",32,"H");
        packet.addControl("hid","crossfader",34,"H");
        packet.addControl("hid","headphones",36,"H");
        packet.addControl("hid","trackpad_x",38,"H");
        packet.addControl("hid","trackpad_y",40,"H");
        packet.addControl("hid","slider_pos_2",42,"H");
        packet.addControl("hid","slider_pos_1",44,"H");
        // Measured: this bit is the TOP-RIGHT corner encoder's push button, not
        // keylock. It was never linked to anything, so the mislabel was
        // harmless, but it left the fourth corner button undeclared.
        packet.addControl("hid","jog_ne_button",46,"I",0x1);
        packet.addControl("hid","beatloop_8",46,"I",0x2);
        packet.addControl("hid","beatloop_4",46,"I",0x4);
        packet.addControl("hid","beatloop_2",46,"I",0x8);
        packet.addControl("hid","beatloop_1",46,"I",0x10);
        packet.addControl("hid","loop_in",46,"I",0x20);
        packet.addControl("hid","loop_out",46,"I",0x40);
        packet.addControl("hid","reloop_exit",46,"I",0x80);
        packet.addControl("hid","slider_scale",46,"I",0x100);
        packet.addControl("hid","jog_se_button",46,"I",0x200);
        packet.addControl("hid","eject_right",46,"I",0x400);
        packet.addControl("hid","deck_switch",46,"I",0x800);
        packet.addControl("hid","eject_left",46,"I",0x1000);
        packet.addControl("hid","jog_sw_button",46,"I",0x2000);
        packet.addControl("hid","stop",46,"I",0x4000);
        packet.addControl("hid","play",46,"I",0x8000);
        packet.addControl("hid","cue",46,"I",0x10000);
        packet.addControl("hid","reverse",46,"I",0x20000);
        packet.addControl("hid","brake",46,"I",0x40000);
        packet.addControl("hid","fastforward",46,"I",0x80000);
        packet.addControl("hid","jog_nw_button",46,"I",0x100000);
        packet.addControl("hid","jog_touch",46,"I",0x200000);
        packet.addControl("hid","trackpad_left",46,"I",0x400000);
        packet.addControl("hid","trackpad_right",46,"I",0x800000);
        packet.addControl("hid","hotcue_1",46,"I",0x1000000);
        packet.addControl("hid","hotcue_2",46,"I",0x2000000);
        packet.addControl("hid","hotcue_3",46,"I",0x4000000);
        packet.addControl("hid","hotcue_4",46,"I",0x8000000);
        packet.addControl("hid","hotcue_5",46,"I",0x10000000);
        packet.addControl("hid","hotcue_6",46,"I",0x20000000);
        packet.addControl("hid","touch_slider",46,"I",0x40000000);
        packet.addControl("hid","touch_trackpad",46,"I",0x80000000);
        packet.addControl("hid","packet_number",51,"B");
        packet.addControl("hid","deck_status",52,"B");
        this.controller.registerInputPacket(packet);
        
        packet = new HIDPacket("firmware_version", 0xa, undefined, [0x0a, 0x04]);
        packet.addControl("hid","major",2,"B");
        packet.addControl("hid","minor",3,"B");
        this.controller.registerInputPacket(packet);

        packet = new HIDPacket("trackpad_mode", 0x5, undefined, [0x05, 0x03]);
        packet.addControl("hid","status",2,"B");
        this.controller.registerInputPacket(packet);


    }

    this.registerOutputPackets = function() {
        var packet = undefined;
        var name = undefined;
        var offset = 0;

        packet = new HIDPacket("button_leds", 0x16, undefined, [0x18]);
        offset = 2; // matches original 2.2.0 raw offset; addOutput()'s internal -1 shim then lands this at data[1], right after the 1-byte header
        packet.addOutput("hid","jog_nw",offset++,"B");
        packet.addOutput("hid","jog_ne",offset++,"B");
        packet.addOutput("hid","jog_se",offset++,"B");
        packet.addOutput("hid","jog_sw",offset++,"B");
        packet.addOutput("hid","beatloop_8",offset++,"B");
        packet.addOutput("hid","beatloop_4",offset++,"B");
        packet.addOutput("hid","beatloop_2",offset++,"B");
        packet.addOutput("hid","beatloop_1",offset++,"B");
        packet.addOutput("hid","loop_in",offset++,"B");
        packet.addOutput("hid","loop_out",offset++,"B");
        packet.addOutput("hid","reloop_exit",offset++,"B");
        packet.addOutput("hid","eject_right",offset++,"B");
        packet.addOutput("hid","deck_switch",offset++,"B");
        packet.addOutput("hid","trackpad_right",offset++,"B");
        packet.addOutput("hid","trackpad_left",offset++,"B");
        packet.addOutput("hid","eject_left",offset++,"B");
        packet.addOutput("hid","stop",offset++,"B");
        packet.addOutput("hid","play",offset++,"B");
        packet.addOutput("hid","reverse",offset++,"B");
        packet.addOutput("hid","cue",offset++,"B");
        packet.addOutput("hid","brake",offset++,"B");
        packet.addOutput("hid","fastforward",offset++,"B");
        packet.length = 31; // match stock 2.2.0's fixed 32-byte total report size (31 + 1 reportID)
        this.controller.registerOutputPacket(packet);

        packet = new HIDPacket("slider_leds", 0x17, undefined, [0x16]);
        offset = 2;
        packet.addOutput("pitch","slider_1",offset++,"B");
        packet.addOutput("pitch","slider_2",offset++,"B");
        packet.addOutput("pitch","slider_3",offset++,"B");
        packet.addOutput("pitch","slider_4",offset++,"B");
        packet.addOutput("pitch","slider_5",offset++,"B");
        packet.addOutput("pitch","slider_6",offset++,"B");
        packet.addOutput("pitch","slider_7",offset++,"B");
        packet.addOutput("pitch","slider_8",offset++,"B");
        packet.addOutput("pitch","slider_9",offset++,"B");
        packet.addOutput("pitch","slider_10",offset++,"B");
        packet.addOutput("pitch","slider_11",offset++,"B");
        packet.addOutput("pitch","slider_12",offset++,"B");
        packet.addOutput("pitch","slider_13",offset++,"B");
        packet.addOutput("pitch","slider_14",offset++,"B");
        packet.addOutput("pitch","slider_15",offset++,"B");
        packet.addOutput("pitch","slider_16",offset++,"B");
        packet.addOutput("pitch","slider_17",offset++,"B");
        packet.addOutput("pitch","slider_scale_1",offset++,"B");
        packet.addOutput("pitch","slider_scale_2",offset++,"B");
        packet.addOutput("pitch","slider_scale_3",offset++,"B");
        packet.length = 31;
        this.controller.registerOutputPacket(packet);

        packet = new HIDPacket("led_wheel_left", 0x14, undefined, [0x20]);
        offset = 2;
        for (var led_index=1;led_index<=this.wheelLEDCount/2;led_index++)
            packet.addOutput("hid","wheel_" + led_index,offset++,"B");
        packet.length = 31;
        this.controller.registerOutputPacket(packet);

        packet = new HIDPacket("led_wheel_right", 0x15, undefined, [0x20]);
        offset = 2;
        for (var led_index=this.wheelLEDCount/2+1;led_index<=this.wheelLEDCount;led_index++)
            packet.addOutput("hid","wheel_" + led_index,offset++,"B");
        packet.length = 31;
        this.controller.registerOutputPacket(packet);

        packet = new HIDPacket("request_firmware_version", 0xa, undefined, [0x2]);
        packet.length = 31;
        this.controller.registerOutputPacket(packet);

        packet = new HIDPacket("set_trackpad_mode", 0x5, undefined, [0x3]);
        packet.addOutput("hid","mode",2,"B");
        packet.length = 31;
        this.controller.registerOutputPacket(packet);

        packet = new HIDPacket("set_ledcontrol_mode", 0x1d, undefined, [0x3]);
        packet.addOutput("hid","mode",2,"B");
        packet.length = 31;
        this.controller.registerOutputPacket(packet);
    }

    // Otus specific output packet to request device firmware version
    this.requestFirmwareVersion = function() {
        var packet = this.controller.getOutputPacket("request_firmware_version");
        if (packet==undefined)
            return;
        HIDDebug("Requesting firmware version " + packet.name);
        packet.send();
    }

    // Set LED Control Mode on Otus firmware versions > 1.6. Major and minor must
    // contain the version numbers for firmware as received from response.
    // Valid modes are:
    //      0   disable all LEDs
    //      1   Re-enable LEDs
    //      2   Revert to built-in light functionality
    this.setLEDControlMode = function(mode) {
        var controller = this.controller;
        if (this.version_major<=1 && this.version_minor<6) {
            // Firmware version does not support LED Control Mode Setting
            return;
        }
        if (mode!=0 && mode!=1 && mode!=2) {
            HIDDebug("Unknown value for LED Control Mode Setting: " + mode);
            return;
        }
        var packet = controller.getOutputPacket("set_ledcontrol_mode");
        var field = packet.getField("hid","mode");
        if (field==undefined) {
            HIDDebug("EksOtus.setLEDControlMode error fetching field mode");
            return;
        }
        field.value = mode;
        packet.send();
    }

    // Firmware version response. Required to finish device INIT
    this.FirmwareVersionResponse = function(packet,delta) {
        var controller = this.controller;
        var field_major = packet.getField("hid","major");
        var field_minor = packet.getField("hid","minor");
        if (field_major==undefined || field_minor==undefined) {
            HIDDebug("Error parsing response version packet");
            return;
        }
        this.version_major = field_major.value;
        this.version_minor = field_minor.value;
        controller.initialized = true;

        this.setLEDControlMode(1);
        if (controller.activeDeck!=undefined) {
            controller.setOutput("hid","deck_switch", controller.LEDColors[controller.deckOutputColors[controller.activeDeck]]);
            controller.switchDeck(controller.activeDeck);
        } else {
            var value = controller.LEDColors["amber"];
            this.controller.setOutputToggle("hid","deck_switch",value);
        }
        this.updateLEDs();
        HIDDebug("EKS " + EksOtus.id +
            " v"+EksOtus.version_major+"."+EksOtus.version_minor+
            " initialized"
        );
    }

    // Otus specific output packet to set the trackpad control mode
    this.setTrackpadMode = function(mode) {
        if (mode!=0 && mode!=1) {
            HIDDebug("Unsupported trackpad mode value: " + mode);
            return;
        }
        var packet = this.controller.getOutputPacket("set_trackpad_mode");
        if (packet==undefined) {
            HIDDebug("Output not registered: set_trackpad_mode");
            return;
        }
        var field = packet.getField("hid","mode");
        if (field==undefined) {
            HIDDebug("EksOtus.setTrackpadMode error fetching field mode");
            return;
        }
        field.value = mode;
        packet.send();
    }

    // Response to above trackpad mode packet
    this.TrackpadModeResponse = function(packet,delta) {
        field = packet.getField("hid","status");
        if (field==undefined) {
            HIDDebug("Error parsing field status from packet");
            return;
        }
        if (field.value==1) {
            HIDDebug("Trackpad mode successfully set");
        } else {
            HIDDebug("Trackpad mode change failed");
        }
    }

    // Generic unsigned short to -1..0..1 range scaling
    this.plusMinus1Scaler = function(group,name,value) {
        if (value<32768)
            return value/32768-1;
        else
            return (value-32768)/32768;
    }

    // Volume slider scaling for 0..1..5 scaling
    this.volumeScaler = function(group,name,value) {
        return script.absoluteNonLin(value, 0, 1, 5, 0, 65536);
    }

    // EQ scaling function for 0..1..4 scaling
    this.eqScaler = function(group,name,value) {
        return script.absoluteNonLin(value, 0, 1, 4, 0, 65536);
    }

    // Mandatory call from init() to initialize hardware
    this.initializeHIDController = function() {
        this.registerInputPackets();
        this.registerOutputPackets();
    }

    this.shutdownHardware = function() {
        this.setLEDControlMode(2);
        this.setTrackpadMode(1);
    }

}

EksOtus = new EKSOtusController();

// Initialize device state, send request for firmware. Otus is not
// usable before we receive a valid firmware version response.
EksOtus.init = function (id) {
    EksOtus.id = id;

    EksOtus.LEDUpdateInterval = 250;
    // Valid values: 1 for mouse mode, 0 for xy-pad mode
    EksOtus.trackpadMode = 0;
    EksOtus.deckSwitchHeld = false;
    EksOtus.deckSwitchHoldPending = false;
    EksOtus.deckSwitchDebounceIgnoring = false;
    // Wheel absolute position value
    EksOtus.wheelPosition = undefined;
    // Absolute platter angle, kept current by wheelDelta for the LED animation.
    EksOtus.wheelAbsolutePosition = 0;
    // Wheel spin animation details
    EksOtus.activeTrackDuration = undefined;
    // Group registered to update spinning platter details
    EksOtus.activeSpinningPlatterGroup = undefined;
    // Virtual record spin time, 1.8 for 33 1/3 RPM, 1.33 for 45 RPM
    EksOtus.revTime = 1.8;
    EksOtus.pitchModifierActive = false;
    // Wheel LED index, range 1-60
    EksOtus.activeSpinningPlatterLED = undefined;

    // Call the HID packet parser initializers
    EksOtus.initializeHIDController();
    var controller = EksOtus.controller;
    // Set callbacks for packets here to avoid issues in callback handling
    controller.setPacketCallback("firmware_version",EksOtus.FirmwareVersionWrapper);
    controller.setPacketCallback("trackpad_mode",EksOtus.TrackpadModeWrapper);

    // NOTE: the engine keys every changed field as "<group>.<name>" (see
    // HIDPacket.parse/parseBitVector in common-hid-packet-parser.js), so an
    // entry here must include the "hid." group prefix to actually match and
    // be skipped. "deck_status", "slider_pos_1", "slider_pos_2", and
    // "slider_value" were missing that prefix and were therefore never
    // actually ignored. In practice the three slider fields have their own
    // setCallback() handler so they short-circuit before this matters, but
    // "deck_status" has none, so its value changes fell through to the
    // generic engine.setValue("hid", "deck_status", ...) fallback - and
    // since "hid" isn't a real Mixxx control group, that logs
    // "ControlDoublePrivate::getControl returning NULL for ("hid","deck_status")".
    controller.ignoredControlChanges = [
        "mask","hid.timestamp","hid.packet_number","hid.deck_status", "hid.wheel_speed",
        // These return the Otus slider position scaled by the 'slider scale'
        // slider_pos_1 is a dead field (constant 0x8000), slider_pos_2 duplicates
        // slider_value, and slider_value itself is unused: pitch is derived from
        // slider_position, which is raw and unaffected by the firmware's divisor.
        "hid.slider_pos_1","hid.slider_pos_2","hid.slider_value"
    ];

    // Scratch parameters
    controller.scratchintervalsPerRev = 1024;
    controller.scratchAlpha = 1.0/8;
    // NOTE: 'rampedScratchEnable' is not a real HIDController property (the
    // engine reads scratchRampOnEnable/scratchRampOnDisable), so this line
    // never had any effect since the script was written - scratch start/stop
    // has always defaulted to an instant jump instead of ramping. Setting
    // the real properties makes engaging/releasing the wheel ramp smoothly,
    // which is part of what should make scratching feel less abrupt/jerky.
    controller.scratchRampOnEnable = true;
    controller.scratchRampOnDisable = true;

    // Use a single continuous scratch curve instead of the 2.2 piecewise one.
    // false = original feel (4x boost below 8 counts/report, with a step
    // change in sensitivity where the two branches meet); true = no step.
    EksOtus.smoothScratch = false;

    // The platter keeps reporting position while you are not touching it, so
    // the stored reference drifts away from where your hand actually lands.
    // Without this, the first report after every touch carries all the
    // movement since the last release and the deck lurches. Dropping the
    // reference on both enable and disable makes the first tick after a touch
    // always 0 and the scratch start from rest.
    controller.enableScratchCallback = function(isScratchEnabled) {
        EksOtus.wheelPosition = undefined;
    };

    EksOtus.setTrackpadMode(this.trackpadMode);
    // Note: Otus is not considered initialized before we get
    // response to this packet
    EksOtus.requestFirmwareVersion();
    // Link controls and register callbacks
    EksOtus.registerCallbacks();

    // CHANGED: headVolume -> headGain for Mixxx 2.5+
    engine.softTakeover("[Master]","headGain",true);
    engine.softTakeover("[Master]","headMix",true);
    for (var deck in controller.deckOutputColors) {
        engine.softTakeover("[Channel"+deck+"]","pregain",true);
        engine.softTakeover("[Channel"+deck+"]","volume",true);
    }

    if (EksOtus.LEDUpdateInterval!=undefined) {
        controller.timers["led_update"] = engine.beginTimer(
            EksOtus.LEDUpdateInterval,
            () => EksOtus.updateLEDs(true)
        );
    }

    EksOtus.startWheelLEDs();

}

// Callback bound to each of the 11 deck-tied outputs below via linkOutput().
// Unchanged from the Mixxx 2.2 original - setOutput("deck", key, ...) only
// resolves correctly now because of the getOutputField compatibility shim
// at the top of this file (Mixxx 2.5's engine broke mapped-identity lookups).
EksOtus.outputCallback = function(value, group, key) {
    var controller = EksOtus.controller;
    if (group=="deck") {
        if (controller.activeDeck==undefined)
            return;
        group = controller.resolveGroup("deck");
    }
    if (value==1)
        EksOtus.controller.setOutput("deck",key,
            controller.LEDColors[controller.deckOutputColors[controller.activeDeck]],
            true
        );
    else
        EksOtus.controller.setOutput("deck",key,controller.LEDColors.off,true);
}

// Mixxx's HIDPacket.send() (common-hid-packet-parser.js) always calls
// controller.sendOutputReport(reportId, data, useNonSkippingFIFO=false).
// With that default, Mixxx's native HID I/O thread silently SKIPS writing
// a report to the device if its bytes are identical to the last data it
// queued for that report ID - a deliberate throughput optimization, not a
// bug (see src/controllers/hid/hidcontroller.h). On this device that skip
// logic appears to also suppress the very first real write, so the
// hardware never receives the report and the LEDs never light. Passing
// useNonSkippingFIFO=true forces every write through unconditionally.
// This re-implements HIDPacket.send()'s packing step locally so we can
// call controller.sendOutputReport() ourselves with that flag set,
// without needing to modify the shared library file.
EksOtus.forceSendPacket = function(packet) {
    // NOTE: do NOT shadow the name "controller" here. Mixxx injects a
    // global "controller" object (the native HidControllerJSProxy with
    // sendOutputReport/send/etc). EksOtus.controller is a DIFFERENT
    // object - our own JS HIDController wrapper instance (getOutputPacket,
    // setOutput, ...) - and has no sendOutputReport method at all.
    var data = new Uint8Array(packet.length);
    if (packet.header !== undefined) {
        for (var header_byte = 0; header_byte < packet.header.length; header_byte++) {
            data[header_byte] = packet.header[header_byte];
        }
    }
    for (var group_name in packet.groups) {
        var group = packet.groups[group_name];
        for (var field_name in group) {
            packet.pack(data, group[field_name]);
        }
    }
    // ---- Cross-platform report framing -----------------------------------
    // This device's HID descriptor declares reportID 0x00 in BOTH directions:
    // it uses NO report IDs. Its wire format is [command, length, payload...]
    // (length = payload + 2: 0x18=24 for the 22 button LEDs, 0x16=22 for the
    // 20 slider LEDs, 0x20=32 for 30 wheel LEDs each).
    //
    // What this mapping calls a packet's "reportId" (0x05, 0x0a, 0x14-0x17,
    // 0x1d) is really that COMMAND byte, not a HID report ID. Passing it to
    // Mixxx as a report ID works on macOS only by accident: Mixxx sends
    // [reportId] + data to hid_write, and hidapi's macOS backend forwards the
    // whole buffer when data[0] != 0, so the command byte reaches the device.
    // The Windows backend always consumes data[0] as the report-ID slot and
    // never transmits it, so every command is silently lost and the device
    // answers with a constant filler blob.
    //
    // Sending report ID 0 with the command byte moved into the data is correct
    // on both: macOS strips the leading zero, Windows puts it in the report-ID
    // slot, and the device receives [command, length, payload...] either way.
    // Verified on hardware (firmware v1.5): this framing makes the LEDs light
    // on Windows for the first time, and [0x00,0x0a,0x02] returns the real
    // firmware-version reply 0A 04 01 05 where the old framing returned junk.
    // WHY THIS MATTERS BEYOND LEDs: with the old framing, Windows stripped the
    // command byte and the device received the REMAINING bytes as a different
    // command. set_trackpad_mode ([0x05][0x03,0x00]) arrived as command 0x03
    // payload 0x00, which the firmware treats as "stop reporting" - and that is
    // the FIRST packet this driver sends at init. Measured: after that one write
    // the controller goes silent until it is physically replugged, which is why
    // the Otus appeared completely dead on Windows. Correctly framed, every
    // command leaves the input stream running.
    var framed = new Uint8Array(packet.length + 1);
    framed[0] = packet.reportId;
    framed.set(data, 1);
    controller.sendOutputReport(0, framed.buffer, true);
}

// Route EVERY output report through the corrected framing above.
//
// forceSendPacket() alone is not enough: the init commands
// (requestFirmwareVersion, setTrackpadMode, setLEDControlMode) call
// packet.send() directly, and HIDController.setOutput(..., true) calls
// field.packet.send() from inside the shared library. Those paths kept using
// the old [reportId] + data framing, which on Windows is what silences the
// controller - so the LEDs worked while the device stayed mute.
//
// Overriding the prototype catches every call site, including the ones inside
// common-hid-packet-parser.js that this file cannot otherwise reach. For
// upstream this belongs in the shared library (or in Mixxx's HID layer) rather
// than as a patch from a mapping, but the behaviour is what matters here.
HIDPacket.prototype.send = function() {
    EksOtus.forceSendPacket(this);
};

EksOtus.updateLEDs = function(from_timer) {
    var controller = EksOtus.controller;
    if (EksOtus.updatePitchLEDs !== undefined) {
        // Refreshed here too, so the bar follows the pitch even when it is
        // changed from the GUI, from sync, or by switching decks.
        EksOtus.updatePitchLEDs();
    }
    EksOtus.forceSendPacket(controller.getOutputPacket("button_leds"));
    EksOtus.forceSendPacket(controller.getOutputPacket("slider_leds"));
    EksOtus.forceSendPacket(controller.getOutputPacket("led_wheel_left"));
    EksOtus.forceSendPacket(controller.getOutputPacket("led_wheel_right"));
}

// Device cleanup function
EksOtus.shutdown = function() {
    // CHANGED: headVolume -> headGain for Mixxx 2.5+
    engine.softTakeover("[Master]","headGain",false);
    engine.softTakeover("[Master]","headMix",false);
    for (var deck in EksOtus.controller.deckOutputColors) {
        engine.softTakeover("[Channel"+deck+"]","pregain",false);
        engine.softTakeover("[Channel"+deck+"]","volume",false);
    }
    EksOtus.shutdownHardware(2);
    HIDDebug("EKS "+EksOtus.id+" shut down");
}

// Mandatory default handler for incoming packets
EksOtus.incomingData = function(data,length) {
    EksOtus.controller.parsePacket(data,length);
}

EksOtus.FirmwareVersionWrapper = function(packet,data) {
    return EksOtus.FirmwareVersionResponse(packet,data);
}

EksOtus.TrackpadModeWrapper = function(packet,data) {
    return EksOtus.TrackpadModeResponse(packet,data);
}

// Callback to set current loaded track's duration for wheel led animation
EksOtus.loadedTrackDuration = function(value) {
    EksOtus.activeTrackDuration = value;
}

// Link virtual HID naming of input and LED controls to mixxx
// Note: HID specification has more fields than we map here.
EksOtus.registerCallbacks = function() {
    var controller = EksOtus.controller;

    controller.modifiers.add("shift");
    controller.modifiers.add("shift");
    controller.linkModifier("hid","eject_right","shift");
    controller.setCallback("control","hid","touch_slider",function(field) { EksOtus.pitchModifierActive = (field.value == 1); });

    controller.linkControl("hid","play","deck","play");
    controller.linkControl("hid","cue","deck","cue_default");
    controller.linkControl("hid","reverse","deck","reverse");
    controller.linkControl("hid","eject_left","deck","pfl");
    controller.linkControl("hid","jog_touch","deck","jog_touch");
    controller.linkControl("hid","wheel_position","deck","jog_wheel");

    controller.linkControl("hid","jog_se_button","deck","LoadSelectedTrack");
    controller.linkControl("hid","jog_se","[Playlist]","SelectTrackKnob");

    controller.linkControl("hid","crossfader","[Master]","crossfader");
    controller.linkControl("hid","gain_1","deck1","pregain");
    controller.linkControl("hid","gain_2","deck2","pregain");
    controller.linkControl("hid","eq_high_1","deck1","filterHigh");
    controller.linkControl("hid","eq_high_2","deck2","filterHigh");
    controller.linkControl("hid","eq_mid_1","deck1","filterMid");
    controller.linkControl("hid","eq_mid_2","deck2","filterMid");
    controller.linkControl("hid","eq_low_1","deck1","filterLow");
    controller.linkControl("hid","eq_low_2","deck2","filterLow");

    controller.setScaler("jog",EksOtus.jogScaler);
    controller.setScaler("jog_scratch",EksOtus.wheelScaler);
    controller.setScaler("SelectTrackKnob",EksOtus.browseScaler);

    EksOtus.registerCornerEncoders();
    controller.setScaler("crossfader",EksOtus.plusMinus1Scaler);
    controller.setScaler("pregain",EksOtus.eqScaler);
    controller.setScaler("filterHigh",EksOtus.eqScaler);
    controller.setScaler("filterMid",EksOtus.eqScaler);
    controller.setScaler("filterLow",EksOtus.eqScaler);

    controller.setCallback("control","hid","hotcue_1",EksOtus.hotcue);
    controller.setCallback("control","hid","hotcue_2",EksOtus.hotcue);
    controller.setCallback("control","hid","hotcue_3",EksOtus.hotcue);
    controller.setCallback("control","hid","hotcue_4",EksOtus.hotcue);
    controller.setCallback("control","hid","hotcue_5",EksOtus.hotcue);
    controller.setCallback("control","hid","hotcue_6",EksOtus.hotcue);

    controller.setCallback("control","hid","beatloop_1",EksOtus.beatloop);
    controller.setCallback("control","hid","beatloop_2",EksOtus.beatloop);
    controller.setCallback("control","hid","beatloop_4",EksOtus.beatloop);
    controller.setCallback("control","hid","beatloop_8",EksOtus.beatloop);
    controller.linkControl("hid","loop_in","deck","loop_in");
    controller.linkControl("hid","loop_out","deck","loop_out");
    controller.linkControl("hid","reloop_exit","deck","reloop_exit");

    controller.setCallback("control","hid","deck_switch",EksOtus.deckSwitch);

    //controller.linkControl("hid","headphones","[Master]","headphones");
    controller.setCallback("control","hid","headphones",EksOtus.headphones);

    controller.setCallback("control","hid","slider_scale",EksOtus.sliderScaleButton);
    controller.setCallback("control","hid","slider_position",EksOtus.pitchSlider);

    controller.linkOutput("hid","beatloop_8","deck","beatloop_8_enabled",EksOtus.outputCallback);
    controller.linkOutput("hid","beatloop_4","deck","beatloop_4_enabled",EksOtus.outputCallback);
    controller.linkOutput("hid","beatloop_2","deck","beatloop_2_enabled",EksOtus.outputCallback);
    controller.linkOutput("hid","beatloop_1","deck","beatloop_1_enabled",EksOtus.outputCallback);
    controller.linkOutput("hid","loop_in","deck","loop_in",EksOtus.outputCallback);
    controller.linkOutput("hid","loop_out","deck","loop_out",EksOtus.outputCallback);
    controller.linkOutput("hid","reloop_exit","deck","reloop_exit",EksOtus.outputCallback);
    controller.linkOutput("hid","eject_left","deck","pfl",EksOtus.outputCallback);
    controller.linkOutput("hid","play","deck","play",EksOtus.outputCallback);
    controller.linkOutput("hid","reverse","deck","reverse",EksOtus.outputCallback);
    controller.linkOutput("hid","cue","deck","cue_default",EksOtus.outputCallback);

}

// wheel_position (packed "H") is a free-running 16-bit counter, so it rolls
// over 65535 -> 0 (and 0 -> 65535 backwards) several times per minute of
// scratching.
//
// Every version of this script up to now handled the rollover by DISCARDING
// the sample ("if (delta>32768) return 0;") and returning *before* updating
// EksOtus.wheelPosition. That left the stored reference stuck on the
// pre-rollover value, so the very next sample produced another >32768 delta,
// which was discarded again... and so on until the platter physically came
// back around to the stale reference. Measured on a steady scratch crossing
// the wrap: 54 of the next 59 packets were thrown away - roughly a full dead
// revolution - then it started working again. That "works, dies, works" cycle
// is the inconsistent scratching.
//
// The correct handling is to UNWRAP the counter (modular arithmetic, both
// directions) rather than discard, and to always resync wheelPosition so the
// reference can never go stale. A separate plausibility limit still drops
// genuinely impossible jumps (unplugged/replugged wheel, dropped report
// bursts), but it resyncs on the way out instead of latching.
EksOtus.wheelModulus = 65536;
// Max believable counter movement in one HID report. The Otus reports far
// faster than a hand can move the platter; anything past this is a glitch.
EksOtus.wheelMaxTick = 2048;

// Returns the signed, unwrapped counter movement since the previous report,
// or 0 when there is no usable reference. Shared by wheelScaler and jogScaler
// so the two can never disagree about where the platter is.
EksOtus.wheelDelta = function(value) {
    if (EksOtus.wheelPosition==undefined) {
        EksOtus.wheelPosition = value;
        EksOtus.wheelAbsolutePosition = value;
        return 0;
    }
    // The platter animation needs the absolute angle, and this runs on every
    // report. Assigned directly rather than through a setter so this function
    // has no dependency on code defined later in the file.
    EksOtus.wheelAbsolutePosition = value;
    var delta = EksOtus.wheelPosition - value;
    var half = EksOtus.wheelModulus/2;
    if (delta > half)
        delta -= EksOtus.wheelModulus;
    else if (delta < -half)
        delta += EksOtus.wheelModulus;
    // Always resync, including on the discard path below.
    EksOtus.wheelPosition = value;
    if (delta > EksOtus.wheelMaxTick || delta < -EksOtus.wheelMaxTick)
        return 0;
    return delta;
}

// Default scaler for jog values
EksOtus.wheelScaler = function(group,name,value) {
    var delta = EksOtus.wheelDelta(value);
    // Kept piecewise to preserve the 2.2 feel: fine movements get 4x the
    // resolution of fast ones. Set EksOtus.smoothScratch = true in
    // EksOtus.init() to use a single continuous curve instead, which removes
    // the 4x step change in sensitivity as you cross +/-8 counts per report.
    if (EksOtus.smoothScratch)
        return -delta/16;
    if (delta>-8 && delta<8)
        return -delta/4;
    return -delta/16;
}

EksOtus.jogScaler = function(group,name,value) {
    return -EksOtus.wheelDelta(value)/64;
}

// The library browse encoder (jog_se) is a free-running 8-bit counter. The
// engine's generic encoder handling only unwraps the exact 255 -> 0 and
// 0 -> 255 steps, so two or more detents between HID reports - trivially easy
// when you spin the encoder to find a track - produce a delta like -253 and
// [Playlist],SelectTrackKnob jumps hundreds of rows in the WRONG direction.
// Measured: spinning forward far enough to advance 33 rows instead moved the
// selection 223 rows backwards. Unwrap into -128..127 and clamp, so a fast
// spin scrolls fast but never teleports or reverses.
EksOtus.browseMaxStep = 8;
EksOtus.browseScaler = function(group,name,delta) {
    delta = ((delta % 256) + 384) % 256 - 128;
    if (delta > EksOtus.browseMaxStep)
        delta = EksOtus.browseMaxStep;
    else if (delta < -EksOtus.browseMaxStep)
        delta = -EksOtus.browseMaxStep;
    return delta;
}

// Deck rate adjustment with top corner wheels
EksOtus.rate_wheel = function(field) {
    var controller = EksOtus.controller;
    if (controller.activeDeck==undefined)
        return;
    var active_group = controller.resolveGroup(field.group);
    var current = engine.getValue(active_group,"rate");
    if (field.delta<0)
        engine.setValue(active_group,"rate",current+0.003);
    else
        engine.setValue(active_group,"rate",current-0.003);
}

// Reset all wheel LEDs to given color. If color is undefined,
// use 'off'
EksOtus.resetWheelLEDs = function (color) {
    var controller = EksOtus.controller;
    if (color==undefined || !(color in controller.LEDColors))
        color = "off";
    // setOutput() needs the numeric LED value, not the color name string
    var color_value = controller.LEDColors[color];
    for (i=1;i<=EksOtus.wheelLEDCount;i++)
        controller.setOutput("hid","wheel_"+i,color_value,false);
    EksOtus.updateLEDs(true);
}

// Rotation of the Otus 'corner' wheels.
// Note right bottom wheel is library browser encoder and not handled here
EksOtus.corner_wheel = function(field) {
    // TODO - attach some functionality these corner wheels!
    print("CORNER " + field.name + " delta " + field.delta);
}

// Hotcues activated with normal press, cleared with shift
EksOtus.hotcue = function (field) {
    var controller = EksOtus.controller;
    var command;
    if (controller.activeDeck==undefined ||
        field.value==controller.buttonStates.released)
        return;
    var active_group = controller.resolveDeckGroup(controller.activeDeck);
    if (controller.modifiers.get("shift"))
        command = field.name + "_clear";
    else
        command = field.name + "_activate";
    engine.setValue(active_group,command,true);
}

// Beatloops activated with normal presses to beatloop_1 - beatloop_8
EksOtus.beatloop = function (field) {
    var controller = EksOtus.controller;
    var command;
    if (controller.activeDeck==undefined ||
        field.value==controller.buttonStates.released)
        return;
    var active_group = controller.resolveDeckGroup(controller.activeDeck);
    command = field.name + "_activate";
    engine.setValue(active_group,command,true);
}

EksOtus.beat_align = function (field) {
    var controller = EksOtus.controller;
    if (controller.activeDeck==undefined)
        return;
    var active_group = controller.resolveGroup(field.group);
    if (controller.modifiers.get("shift")) {
        // if (field.value==controller.buttonStates.released) return;
        engine.setValue(active_group,"beats_translate_curpos",field.value);
    } else {
        if (field.value==controller.buttonStates.released)
            return;
        if (!engine.getValue(active_group,"quantize"))
            engine.setValue(active_group,"quantize",true);
        else
            engine.setValue(active_group,"quantize",false);
    }
}


// Set pregain, if modifier shift is active, deck volume otherwise
EksOtus.volume_pregain = function (field) {
    var controller = EksOtus.controller;
    if (controller.activeDeck==undefined)
        return;
    var active_group = controller.resolveGroup(field.group);
    if (controller.modifiers.get("shift")) {
        value = script.absoluteNonLin(field.value, 0, 1, 5, 0, 65536);
        engine.setValue(active_group,"pregain",value);
    } else {
        value = field.value / 65536;
        engine.setValue(active_group,"volume",value);
    }
}

// Set headphones volume, if modifier shift is active, pre/main mix otherwise
EksOtus.headphones = function (field) {
    var controller = EksOtus.controller;
    if (controller.modifiers.get("shift")) {
        value = script.absoluteNonLin(field.value, 0, 1, 5, 0, 65536);
        // CHANGED: headVolume -> headGain for Mixxx 2.5+
        engine.setValue("[Master]","headGain",value);
    } else {
        value = EksOtus.plusMinus1Scaler(field.group,field.name,field.value);
        engine.setValue("[Master]","headMix",value);
    }
}

// Control effects or something with XY pad
EksOtus.xypad = function(field) {
    var controller = EksOtus.controller;
    if (controller.activeDeck==undefined)
        return;
    print ("XYPAD group " + field.group +
        " name " + field.name + " value " + field.value
    );
}

// How long (ms) 'deck_switch' must be held before it is treated as a
// long press that temporarily switches deck controls until released.
EksOtus.deckSwitchHoldTime = 400;

// How long (ms) to ignore a new 'deck_switch' press right after a tap was
// already handled. A real hardware log capture showed a single physical
// tap being reported as TWO complete press/release cycles back-to-back
// (switch contact bounce) - each one is individually a valid, fast tap, so
// the debounce guard above alone can't tell them apart. The first cycle
// switches decks, the second (spurious) cycle immediately switches back,
// so nothing visibly changes (or - as observed - the LED flashes the new
// color for an instant and then flips right back). 150ms was not always
// enough: on some taps the bounce runs a bit longer than that, so the
// trailing bounce cycle arrives just after the window closed and gets
// treated as a brand-new tap, switching the deck straight back. Widened
// to 300ms, which still leaves 100ms of clearance under deckSwitchHoldTime
// (400ms) so it can't interfere with a genuine hold.
EksOtus.deckSwitchDebounceTime = 300;
EksOtus.deckSwitchDebounceUntil = 0;

// Function called when the special 'Deck Switch' button is pressed
// TODO - add code for 'hold deck_switch and press hot_cue[1-4]
// to select deck 1-4
//
// Behaviour:
// - quick press/release (tap) -> persistent deck switch
// - press and hold for deckSwitchHoldTime -> temporary deck switch,
//   reverted automatically when the button is released
EksOtus.deckSwitch = function(field) {
    var controller = EksOtus.controller;
    if (EksOtus.initialized==false)
        return;

    if (field.value == controller.buttonStates.pressed) {
        if (Date.now() < EksOtus.deckSwitchDebounceUntil) {
            // Spurious repeated press (contact bounce) right after a tap
            // was already handled - ignore this whole press/release cycle.
            // Slide the window forward on every bounce we see so a longer
            // or multi-cycle bounce burst is fully swallowed instead of
            // only the first extra cycle.
            EksOtus.deckSwitchDebounceIgnoring = true;
            EksOtus.deckSwitchDebounceUntil = Date.now() + EksOtus.deckSwitchDebounceTime;
            HIDDebug("EksOtus.deckSwitch - ignoring bounced press");
            return;
        }
        EksOtus.deckSwitchDebounceIgnoring = false;
        // Start the long-press timer. If it fires while the button is
        // still held, EksOtus.deckSwitchHoldTrigger() will temporarily
        // switch decks. deckSwitchHoldPending is an explicit guard flag,
        // checked inside the timer callback itself - engine.stopTimer()
        // alone was not reliably cancelling this timer, so a released tap
        // was cleared from controller.timers but the timer fired anyway
        // ~deckSwitchHoldTime later and reverted the deck, silently
        // cancelling the tap's switch. The flag makes a late/duplicate
        // firing a guaranteed no-op regardless of whether stopTimer worked.
        EksOtus.deckSwitchHoldPending = true;
        controller.timers["deck_switch_hold"] = engine.beginTimer(
            EksOtus.deckSwitchHoldTime, EksOtus.deckSwitchHoldTrigger, true
        );
        return;
    }

    // field.value == controller.buttonStates.released
    if (EksOtus.deckSwitchDebounceIgnoring) {
        // Release half of a bounced press we already ignored above.
        EksOtus.deckSwitchDebounceIgnoring = false;
        return;
    }

    if (EksOtus.deckSwitchHoldPending) {
        // Released before the hold threshold - this was a quick tap,
        // switch decks persistently. Clear the pending flag FIRST so that
        // even if the hold timer still fires later (stopTimer unreliable),
        // EksOtus.deckSwitchHoldTrigger() will see it is no longer pending
        // and do nothing.
        EksOtus.deckSwitchHoldPending = false;
        if (controller.timers["deck_switch_hold"] != undefined) {
            engine.stopTimer(controller.timers["deck_switch_hold"]);
            delete controller.timers["deck_switch_hold"];
        }
        EksOtus.deckSwitchPersistent();
        EksOtus.deckSwitchDebounceUntil = Date.now() + EksOtus.deckSwitchDebounceTime;
        return;
    }

    if (EksOtus.deckSwitchHeld) {
        // The long press already triggered a temporary deck switch -
        // releasing the button reverts back to the original deck.
        EksOtus.deckSwitchHeld = false;
        controller.switchDeck();
        controller.setOutput("hid","deck_switch", controller.LEDColors[controller.deckOutputColors[controller.activeDeck]]);
        EksOtus.updateLEDs();
        EksOtus.deckSwitchDebounceUntil = Date.now() + EksOtus.deckSwitchDebounceTime;
        HIDDebug("Active EKS Otus deck reverted to " + controller.activeDeck);
    }
}

// Timer callback fired when 'deck_switch' has been held down continuously
// for EksOtus.deckSwitchHoldTime - temporarily switches deck controls
// until the button is released.
EksOtus.deckSwitchHoldTrigger = function() {
    var controller = EksOtus.controller;
    delete controller.timers["deck_switch_hold"];
    if (!EksOtus.deckSwitchHoldPending) {
        // The button was already released (a quick tap) and handled -
        // this firing is stale/late, ignore it.
        return;
    }
    EksOtus.deckSwitchHoldPending = false;
    EksOtus.deckSwitchHeld = true;
    controller.switchDeck();
    controller.setOutput("hid","deck_switch", controller.LEDColors[controller.deckOutputColors[controller.activeDeck]]);
    EksOtus.updateLEDs();
    HIDDebug("Active EKS Otus deck temporarily switched to " + controller.activeDeck);
}

// Function to handle a quick tap of 'deck_switch' - switches decks
// persistently (stays switched until tapped or held again).
EksOtus.deckSwitchPersistent = function() {
    var controller = EksOtus.controller;
    controller.switchDeck();
    controller.setOutput("hid","deck_switch", controller.LEDColors[controller.deckOutputColors[controller.activeDeck]]);
    EksOtus.updateLEDs();
    HIDDebug("Active EKS Otus deck now " + controller.activeDeck);
}

// Switch the visual LED feedback on platter LEDs to active deck
// NOTE this is now disabled, it causes HID input errors in firmware!
EksOtus.activateSpinningPlatterLEDs = function() {
    var controller = EksOtus.controller;
    var active_group = controller.resolveDeckGroup(controller.activeDeck);
    if (active_group==undefined)
        return;
    if (!(controller.activeDeck in controller.deckOutputColors)) {
        HIDDebug("LED color not mapped to deck " % controller.activeDeck);
        return;
    }
    if (active_group==undefined) {
        EksOtus.disableSpinningPlatterLEDs();
        return;
    }
    if (EksOtus.activeSpinningPlatterGroup !=undefined) {
        EksOtus.disableSpinningPlatterLEDs();
    }
    EksOtus.activeSpinningPlatterGroup = active_group;
    EksOtus.loadedTrackDuration(engine.getValue(active_group,"duration"));
    EksOtus.enableSpinningPlatterLEDs();
}

// Enable spinning platter LED functionality for active virtual deck
EksOtus.enableSpinningPlatterLEDs = function() {
    if (EksOtus.wheelLEDsEnabled) {
        // Superseded by the timer-driven animation at the end of this file,
        // which also handles the jog. Leaving both wired would mean two things
        // writing the wheel packets from different sources.
        return;
    }
    if (EksOtus.activeSpinningPlatterGroup==undefined)
        return;
    engine.makeConnection(
        EksOtus.activeSpinningPlatterGroup,
        "playposition",
        "EksOtus.circleLEDs"
    );
    engine.makeConnection(
        EksOtus.activeSpinningPlatterGroup,
        "duration",
        "EksOtus.loadedTrackDuration"
    )
    EksOtus.resetWheelLEDs("off",false);
}

// Disable spinning platter LED functionality for active virtual deck
EksOtus.disableSpinningPlatterLEDs = function() {
    if (EksOtus.activeSpinningPlatterGroup==undefined)
        return;
    engine.makeConnection(
        EksOtus.activeSpinningPlatterGroup,
        "playposition",
        "EksOtus.circleLEDs",
        true
    );
    engine.makeConnection(
        EksOtus.activeSpinningPlatterGroup,
        "duration",
        "EksOtus.loadedTrackDuration",
        true
    )
    EksOtus.resetWheelLEDs("off",false);
}

// Callback from engine to set every third LED in circling pattern according to
// the track position. Careful not to enable sending all 60 positions, it may
// cause too much HID traffic!
EksOtus.circleLEDs = function(position) {
    var controller = EksOtus.controller;
    if (position<0 || position>1) {
        EksOtus.resetWheelLEDs("off",false);
        return;
    }
    // Only update every third LED to save HID packet bandwidth
    var wheelLEDSplit = 3;
    var wheelLEDGroups = EksOtus.wheelLEDCount/wheelLEDSplit;
    var timeRemaining = ((1-position)*EksOtus.activeTrackDuration) | 0;
    var track_pos = position * EksOtus.activeTrackDuration;
    var revolutions = track_pos / EksOtus.revTime;
    var led_index = (((revolutions-(revolutions|0))*wheelLEDGroups)|0)*wheelLEDSplit;
    led_index++;
    if (led_index==EksOtus.activeSpinningPlatterLED)
        return;
    EksOtus.activeSpinningPlatterLED = led_index;
    EksOtus.resetWheelLEDs("off",false);
    // setOutput() needs the numeric LED value, not the color name string
    var led_color = controller.LEDColors[controller.deckOutputColors[controller.activeDeck]];
    controller.setOutput("hid","wheel_"+(led_index),led_color);
    EksOtus.updateLEDs();
}

// ===========================================================================
// PITCH SLIDER: accuracy button + LED bar
//
// All of this is measured on hardware (firmware v1.5), not inferred:
//
//   slider_position  absolute finger position on the strip, 0..65535, and
//                    exactly 0 whenever the strip is not touched.
//   slider_value     a firmware accumulator: value = base + position/divisor,
//                    where base advances on release. It is 16-bit and DOES
//                    saturate - after a few one-directional sweeps at high
//                    sensitivity it pins and stops moving entirely. Pitch is
//                    therefore derived from slider_position, never from this.
//   slider_pos_1     constant 0x8000 in 275 consecutive samples. Dead field.
//   slider_pos_2     byte-identical to slider_value. Duplicate, not a second
//                    touch point - the strip is not multitouch.
//   slider_scale     offset 46 mask 0x100 (the mapping had this right all
//                    along; it was simply wired to a callback that ignored it).
//
// The ACCURACY IS IMPLEMENTED IN THE FIRMWARE, not here. Pressing the scale
// button cycles the divisor it applies, measured over ~2400 samples as a clean
// period-3 cycle:
//
//      /32  ->  /2  ->  /8  ->  /32 ...
//
// i.e. sensitivities of 1x, 16x and 4x. So this script must NOT apply a
// multiplier of its own on top of slider_value, or the two scalings compound.
// It tracks which state the firmware is in so it can light the matching
// indicator LED, and scales its own reading of slider_position to match.
// ===========================================================================

// The firmware cycles its own divisor /32 -> /2 -> /8 on each press, but that
// only scales slider_value, which nothing here reads. Recorded for reference.
EksOtus.sliderScaleIndex = 0;

// How far one full sweep of the strip moves the pitch, in rate units, per
// setting, IN PRESS ORDER.
//
// Mixxx's "rate" control spans -1 .. +1, i.e. 2.0 units end to end, so 2.0 here
// means a single sweep of the strip travels the ENTIRE pitch fader in the
// software - bottom of the strip puts the fader at one end, top at the other.
// That is the first setting. Each press then halves the travel, so the strip
// gets progressively finer rather than coarser.
//
// The firmware's own divisor only scales slider_value, which nothing here
// reads; pitch comes from slider_position, which is raw. So these numbers are
// exact rather than fighting a hardware scaling we cannot see.
EksOtus.pitchSweepRange = [2.0, 1.0, 0.5];

// Overall trim applied on top, if the whole range wants nudging.
EksOtus.pitchStripGain = 1.0;

// Light the LED inside the scale button whenever the pitch is off zero, as a
// "pitch is engaged" cue. Off by default - its intended meaning is unknown.
EksOtus.scaleButtonLEDShowsPitchActive = true;

// Blank the bar when the deck is stopped.
EksOtus.pitchLEDsOnlyWhilePlaying = true;

// slider_1 is at the TOP of the strip (confirmed by walking the LEDs), so the
// bar is mirrored: segment 1 of the pitch range must light the BOTTOM LED.
EksOtus.sliderLEDsReversed = true;

EksOtus.sliderLastPosition = undefined;

// Sensitivity of the current state relative to the finest one.
// Rate units per count of strip movement at the current setting.
EksOtus.sliderSensitivity = function() {
    return EksOtus.pitchSweepRange[EksOtus.sliderScaleIndex] / 65536;
};

// Scale button.
//
//   short press  cycle the pitch accuracy (1x -> 2x -> 4x)
//   long press   reset the deck's pitch to zero
//
// The button reports a HELD LEVEL rather than a pulse - verified on the unit,
// with deliberate holds timing at 2079 ms and 2463 ms - so the duration is
// genuinely observable. Both actions fire on RELEASE, because a short press
// cannot be distinguished from a long one until the button comes back up.
//
// The firmware advances its own divisor on every press regardless of how long
// it is held. That no longer matters: the divisor only scales slider_value,
// which nothing here reads, and the travel is owned by pitchSweepRange.
EksOtus.scaleLongPressMs = 500;
EksOtus.sliderScalePressedAt = undefined;

EksOtus.sliderScaleButton = function(field) {
    var controller = EksOtus.controller;
    if (field.value === 1) {
        EksOtus.sliderScalePressedAt = Date.now();
        return;
    }
    if (EksOtus.sliderScalePressedAt === undefined) {
        return;   // a release with no press we saw
    }
    var held = Date.now() - EksOtus.sliderScalePressedAt;
    EksOtus.sliderScalePressedAt = undefined;

    if (held >= EksOtus.scaleLongPressMs) {
        if (controller.activeDeck !== undefined) {
            engine.setValue(controller.resolveDeckGroup(controller.activeDeck), "rate", 0);
            // Drop the strip reference so the next touch starts from the new
            // zero rather than carrying on from where the finger last was.
            EksOtus.sliderLastPosition = undefined;
        }
    } else {
        EksOtus.sliderScaleIndex =
            (EksOtus.sliderScaleIndex + 1) % EksOtus.pitchSweepRange.length;
    }
    EksOtus.updatePitchLEDs();
};

// Pitch strip. Driven by slider_position deltas rather than slider_value,
// because slider_value saturates; position is absolute and cannot.
EksOtus.pitchSlider = function(field) {
    var controller = EksOtus.controller;
    if (controller.activeDeck === undefined) {
        return;
    }
    if (field.name !== "slider_position") {
        return;
    }
    var position = field.value;
    // 0 means "not touched" - drop the reference so the next touch starts from
    // rest instead of jumping by everything that happened in between.
    if (position === 0) {
        EksOtus.sliderLastPosition = undefined;
        return;
    }
    if (EksOtus.sliderLastPosition === undefined) {
        EksOtus.sliderLastPosition = position;
        return;
    }
    var delta = position - EksOtus.sliderLastPosition;
    EksOtus.sliderLastPosition = position;
    if (delta === 0) {
        return;
    }
    var group = controller.resolveDeckGroup(controller.activeDeck);
    var rate = engine.getValue(group, "rate");
    rate += delta * EksOtus.sliderSensitivity() * EksOtus.pitchStripGain;
    if (rate > 1) { rate = 1; } else if (rate < -1) { rate = -1; }
    engine.setValue(group, "rate", rate);
    EksOtus.updatePitchLEDs();
};

// Pitch bar plus the accuracy indicators.
//
// PHYSICAL LAYOUT, mapped by lighting each output in turn on the unit:
//
//   16 LEDs alongside the pitch strip   -> slider_1 .. slider_16
//                                          slider_1 is at the TOP
//    3 LEDs above the scale button      -> slider_17, slider_scale_1, slider_scale_2
//    1 LED inside the scale button      -> slider_scale_3
//
// 16 + 3 + 1 = 20, exactly the number of bytes in the slider_leds packet, so
// every output is accounted for. Note the three accuracy indicators are offset
// by one from their names: driving slider_scale_1/2/3 lights the SECOND and
// THIRD indicators and then the LED inside the button, skipping the first -
// which is why the first indicator is addressed as slider_17.
//
// Lit in the ACTIVE DECK's colour, as every other LED on this unit is (see
// deckOutputColors / outputCallback). Drawn as a bar from the centre out to the
// current pitch, so direction and magnitude read off one colour.
EksOtus.pitchBarLEDs = ["slider_1","slider_2","slider_3","slider_4","slider_5",
                        "slider_6","slider_7","slider_8","slider_9","slider_10",
                        "slider_11","slider_12","slider_13","slider_14",
                        "slider_15","slider_16"];
EksOtus.pitchUnusedLEDs = [];
EksOtus.scaleIndicatorLEDs = ["slider_17","slider_scale_1","slider_scale_2"];
EksOtus.scaleButtonLED = "slider_scale_3";

EksOtus.updatePitchLEDs = function() {
    var controller = EksOtus.controller;
    if (controller.activeDeck === undefined) {
        return;
    }
    var group = controller.resolveDeckGroup(controller.activeDeck);
    var deckColor = controller.LEDColors[controller.deckOutputColors[controller.activeDeck]];
    var off = controller.LEDColors.off;
    var bar = EksOtus.pitchBarLEDs;
    var count = bar.length;

    var rate = engine.getValue(group, "rate");
    if (rate === undefined) { rate = 0; }
    if (rate > 1) { rate = 1; } else if (rate < -1) { rate = -1; }

    // The bar reports the pitch of a moving track, so it is meaningless on a
    // stopped deck - blank it rather than leaving a stale reading lit.
    var playing = !EksOtus.pitchLEDsOnlyWhilePlaying ||
                  engine.getValue(group, "play") > 0;

    // rate -1..1 -> 1..count, then mirrored because slider_1 is at the top.
    var segment = Math.round((rate + 1) / 2 * (count - 1)) + 1;
    if (segment < 1) { segment = 1; } else if (segment > count) { segment = count; }
    if (EksOtus.sliderLEDsReversed) {
        segment = count + 1 - segment;
    }
    // A single LED marks the position - no bar from centre.
    for (var i = 0; i < count; i++) {
        controller.setOutput("pitch", bar[i],
            (playing && (i + 1) === segment) ? deckColor : off, false);
    }
    for (var u = 0; u < EksOtus.pitchUnusedLEDs.length; u++) {
        controller.setOutput("pitch", EksOtus.pitchUnusedLEDs[u], off, false);
    }
    // Blanked with the bar when the deck is stopped, so the whole pitch area
    // goes dark together rather than leaving the indicators floating.
    for (var sc = 0; sc < EksOtus.scaleIndicatorLEDs.length; sc++) {
        controller.setOutput("pitch", EksOtus.scaleIndicatorLEDs[sc],
            (playing && sc === EksOtus.sliderScaleIndex) ? deckColor : off, false);
    }
    if (EksOtus.scaleButtonLED !== undefined) {
        var buttonLit = EksOtus.scaleButtonLEDShowsPitchActive &&
                        playing && Math.abs(rate) > 0.001;
        controller.setOutput("pitch", EksOtus.scaleButtonLED,
            buttonLit ? deckColor : off, false);
    }
};

// ===========================================================================
// PLATTER LEDs
//
// 60 LEDs around the wheel, wheel_1 .. wheel_60, split across two output
// packets (led_wheel_left = 1..30, led_wheel_right = 31..60).
//
//   deck playing, jog untouched   one LED circles at 33 1/3 RPM
//   jog touched                   two clusters of three, 180 degrees apart,
//                                 following the platter
//   otherwise                     dark
//
// wheel_position is an ABSOLUTE 16-bit angle: one full revolution spans the
// whole 0..65535 range. Measured by turning the platter exactly two turns and
// unwrapping: 65352 counts, i.e. 2^16 per revolution to within the accuracy of
// stopping by hand. So the jog animation reads the angle directly - no
// accumulator, and it cannot drift out of step with the physical platter.
//
// Playback angle comes from playposition * duration / revTime rather than from
// a clock, so pitch changes, scratching and seeking are all reflected without
// reading any further controls.
// ===========================================================================

EksOtus.wheelLEDsEnabled = true;
EksOtus.wheelCountsPerRev = 65536;
EksOtus.jogClusterSize = 3;          // LEDs per cluster
EksOtus.jogClusterCount = 2;         // clusters, spread evenly around the circle
EksOtus.wheelLEDIntervalMs = 30;     // how often the pattern is recomputed
// Move in steps of this many LEDs, so a revolution has 60/step positions.
//
// 1 uses all 60 and is the smoothest. Raising it only reduces how many HID
// packets the animation sends - the LED still travels at the same speed, so at
// 33 1/3 RPM the visual difference between 1 and 2 is imperceptible. Raise it
// only if the motion ever stutters, which it does not on this hardware.
EksOtus.wheelLEDStep = 1;

EksOtus.wheelLEDState = undefined;   // last rendered pattern, to avoid redundant writes
EksOtus.wheelAbsolutePosition = 0;

// Which LEDs should be lit, as an array of 1-based indices, or [] for dark.
EksOtus.wheelLEDPattern = function() {
    var controller = EksOtus.controller;
    if (controller.activeDeck === undefined) {
        return [];
    }
    var group = controller.resolveDeckGroup(controller.activeDeck);
    var count = EksOtus.wheelLEDCount;
    var step = EksOtus.wheelLEDStep;

    // Quantise a 0..1 turn to a lit LED index, snapped to the step.
    var snap = function(turn) {
        var led = Math.floor(turn * count);
        return Math.floor(led / step) * step;
    };
    var wrap = function(i) {
        return (((i % count) + count) % count) + 1;
    };

    if (controller.isScratchEnabled) {
        // Jog in use: clusters follow the platter's absolute angle. The cluster
        // POSITION is stepped, but its LEDs stay adjacent so each mark reads as
        // a solid block rather than a dotted one.
        var turn = (EksOtus.wheelAbsolutePosition % EksOtus.wheelCountsPerRev) /
                   EksOtus.wheelCountsPerRev;
        var base = snap(turn);
        var lit = [];
        for (var c = 0; c < EksOtus.jogClusterCount; c++) {
            var centre = base + Math.round(c * count / EksOtus.jogClusterCount);
            for (var k = 0; k < EksOtus.jogClusterSize; k++) {
                lit.push(wrap(centre + k - ((EksOtus.jogClusterSize - 1) >> 1)));
            }
        }
        return lit;
    }

    if (engine.getValue(group, "play") > 0) {
        var duration = engine.getValue(group, "duration");
        var position = engine.getValue(group, "playposition");
        if (!duration || position === undefined || position < 0) {
            return [];
        }
        var revolutions = (position * duration) / EksOtus.revTime;
        return [wrap(snap(revolutions - Math.floor(revolutions)))];
    }
    return [];
};

EksOtus.renderWheelLEDs = function() {
    if (!EksOtus.wheelLEDsEnabled) {
        return;
    }
    var controller = EksOtus.controller;
    if (controller.activeDeck === undefined) {
        return;
    }
    var lit = EksOtus.wheelLEDPattern();

    // Skip the write entirely when nothing changed - at 33 updates/sec this is
    // the difference between a trickle of HID traffic and a flood.
    var key = lit.join(",");
    if (key === EksOtus.wheelLEDState) {
        return;
    }
    EksOtus.wheelLEDState = key;

    var color = controller.LEDColors[controller.deckOutputColors[controller.activeDeck]];
    var off = controller.LEDColors.off;
    var isLit = {};
    for (var i = 0; i < lit.length; i++) {
        isLit[lit[i]] = true;
    }
    for (var n = 1; n <= EksOtus.wheelLEDCount; n++) {
        controller.setOutput("hid", "wheel_" + n, isLit[n] ? color : off, false);
    }
    // Only the two wheel packets - not the button or slider banks.
    EksOtus.forceSendPacket(controller.getOutputPacket("led_wheel_left"));
    EksOtus.forceSendPacket(controller.getOutputPacket("led_wheel_right"));
};

EksOtus.startWheelLEDs = function() {
    if (!EksOtus.wheelLEDsEnabled || EksOtus.wheelLEDIntervalMs === undefined) {
        return;
    }
    EksOtus.controller.timers["wheel_leds"] = engine.beginTimer(
        EksOtus.wheelLEDIntervalMs,
        function() { EksOtus.renderWheelLEDs(); }
    );
};

// ===========================================================================
// CORNER ENCODERS
//
// Four encoders, one per corner, each with a push button and an LED. Mapped to
// corners by turning and pressing each in isolation and watching which packet
// offset and which bit moved:
//
//   corner        turn      push bit     assigned here
//   top-left      off 17    0x100000     headphone volume      / push: unity
//   top-right     off 14    0x1          library page scroll   / push: open item
//   bottom-left   off 16    0x2000       crates and playlists  / push: switch pane
//   bottom-right  off 15    0x200        track select          / push: load track
//
// Only the bottom-right was wired before; the other three were decoded and
// discarded, and offsets 14 and 17 shared the name "rate_encoder" so one of
// them was unreachable entirely.
// ===========================================================================

// All four are free-running 8-bit counters with the same wrap flaw as the
// library encoder: the engine only unwraps the exact 255<->0 step, so spinning
// quickly produces deltas like -253. Unwrap into -128..127 and clamp.
EksOtus.cornerMaxStep = 8;
EksOtus.cornerDelta = function(delta) {
    delta = ((delta % 256) + 384) % 256 - 128;
    if (delta > EksOtus.cornerMaxStep) {
        delta = EksOtus.cornerMaxStep;
    } else if (delta < -EksOtus.cornerMaxStep) {
        delta = -EksOtus.cornerMaxStep;
    }
    return delta;
};

// --- top-left: headphone volume -------------------------------------------
// Adjusted in normalised 0..1 parameter space so the control's real range does
// not have to be hard-coded. The steps are small enough that softTakeover
// (enabled for headGain in init) passes them straight through.
EksOtus.headGainStep = 0.02;
EksOtus.cornerHeadGain = function(field) {
    var delta = EksOtus.cornerDelta(field.delta);
    if (delta === 0) {
        return;
    }
    var value = engine.getParameter("[Master]", "headGain") + delta * EksOtus.headGainStep;
    if (value < 0) { value = 0; } else if (value > 1) { value = 1; }
    engine.setParameter("[Master]", "headGain", value);
};
EksOtus.cornerHeadGainReset = function(field) {
    if (field.value === 1) {
        engine.setValue("[Master]", "headGain", 1.0);   // unity
    }
};

// --- top-right: scroll the library a page at a time ------------------------
EksOtus.cornerScroll = function(field) {
    var delta = EksOtus.cornerDelta(field.delta);
    if (delta !== 0) {
        engine.setValue("[Library]", "ScrollVertical", delta);
    }
};
EksOtus.cornerGoToItem = function(field) {
    if (field.value === 1) {
        engine.setValue("[Library]", "GoToItem", 1);
    }
};

// --- bottom-left: move between crates and playlists -------------------------
EksOtus.cornerPlaylist = function(field) {
    var delta = EksOtus.cornerDelta(field.delta);
    if (delta !== 0) {
        engine.setValue("[Playlist]", "SelectPlaylist", delta);
    }
};
// Push jumps focus between the crate sidebar and the track list - the natural
// companion to browsing crates, and it avoids duplicating GoToItem above.
EksOtus.cornerMoveFocus = function(field) {
    if (field.value === 1) {
        engine.setValue("[Library]", "MoveFocusForward", 1);
    }
};

EksOtus.registerCornerEncoders = function() {
    var controller = EksOtus.controller;
    controller.setCallback("control", "hid", "jog_nw", EksOtus.cornerHeadGain);
    controller.setCallback("control", "hid", "jog_ne", EksOtus.cornerScroll);
    controller.setCallback("control", "hid", "jog_sw", EksOtus.cornerPlaylist);
    // jog_se stays linked to [Playlist],SelectTrackKnob in registerCallbacks().

    controller.setCallback("control", "hid", "jog_nw_button", EksOtus.cornerHeadGainReset);
    controller.setCallback("control", "hid", "jog_ne_button", EksOtus.cornerGoToItem);
    controller.setCallback("control", "hid", "jog_sw_button", EksOtus.cornerMoveFocus);
    // jog_se_button stays linked to LoadSelectedTrack.
};
