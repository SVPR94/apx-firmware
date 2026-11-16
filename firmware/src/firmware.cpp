// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX flight firmware - portable application (bring-up milestone 1).
//
// Powers the board, blinks the status LED and runs APX's parameter, MAVLink
// and mode code over the board's GCS link so a ground station can connect.
// There are no sensor drivers and no motor outputs yet, and the GCS is told
// so: no sensors present, no attitude or position, battery unknown, and
// arming is refused.

#include "apx/firmware/firmware.hpp"

#include <cstdio>

#include "apx/firmware/board.hpp"
#include "apx/kernel/periodic.hpp"
#include "apx/kernel/timer.hpp"
#include "io_coprocessor.hpp"
#include "mavlink_handler.hpp"
#include "mode_manager.hpp"

namespace apx::firmware {
namespace {

// MavlinkHandler byte writer -> board GCS link.
void link_write(const uint8_t* data, size_t len, void* ctx) {
    static_cast<apx::hal::IHalUart*>(ctx)->write(data, len);
}

void report_bringup_state(apx::gcs::MavlinkHandler& mav) {
    mav.set_vehicle_type(MAV_TYPE_HEXAROTOR);
    mav.set_sensor_status(0, 0, 0);
    mav.set_estimates_valid(false, false);
    mav.set_arming_blocked("Arming refused: APX bring-up build has no sensors or motor outputs yet");
    mav.set_vehicle_state(0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                          -1.0f, -1.0f, -1, false, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0);
    mav.set_raw_sensor_telemetry(apx::gcs::RawSensorTelemetry{}); // GPS_RAW_INT: no GPS
}

} // namespace

int run() {
    apx::board::init();
    apx::board::set_status_led(false);

    static apx::gcs::MavlinkHandler mav(1, 1);
    mav.init(); // registers the parameter defaults
    static apx::modes::ModeManager modes;
    mav.set_mode_manager(&modes);
    report_bringup_state(mav);

    apx::hal::IHalUart* link = apx::board::gcs_link();
    if (link != nullptr) {
        mav.set_byte_writer(link_write, link);
    }
    mav.set_reboot_supported(true); // MAVLink reboot / reboot-to-bootloader

    // IO coprocessor (boards whose main outputs / RC input sit on an IO MCU).
    // Outputs stay disabled: the IO MCU keeps its own safe values.
    static apx::drivers::IoCoprocessor io;
    io.begin(apx::board::iomcu_link());
    auto io_state = io.state();
    bool io_safety_off = false;
    uint32_t next_rc_report_ms = 0;

    apx::kernel::PeriodicTimer tick(kLoopPeriodUs);
    tick.start();

    uint32_t loops = 0;
    uint64_t next_notice_ms = 3000;
    while (true) {
        tick.wait();

        if (link != nullptr) {
            uint8_t buf[128];
            int got;
            while ((got = link->read(buf, sizeof(buf))) > 0) {
                mav.receive_bytes(buf, static_cast<size_t>(got));
            }
        }

        mav.set_custom_mode(apx::modes::ModeManager::to_custom_mode_number(modes.current_id()));
        mav.update();

        const uint32_t now_ms = static_cast<uint32_t>(apx::kernel::now_ms());
        io.update(now_ms);
        if (io.state() != io_state) {
            io_state = io.state();
            char text[50];
            if (io_state == apx::drivers::IoCoprocessor::State::Ready) {
                std::snprintf(text, sizeof(text), "IO coprocessor ready (protocol %u/%u)",
                              io.protocol_version(), io.protocol_version2());
            } else {
                std::snprintf(text, sizeof(text), "IO coprocessor: %s",
                              apx::drivers::IoCoprocessor::state_name(io_state));
            }
            mav.send_statustext(io_state == apx::drivers::IoCoprocessor::State::Ready ? MAV_SEVERITY_INFO
                                                                                      : MAV_SEVERITY_WARNING, text);
        }
        if (io_state == apx::drivers::IoCoprocessor::State::Ready) {
            if (io.status().safety_off != io_safety_off) {
                io_safety_off = io.status().safety_off;
                mav.send_statustext(MAV_SEVERITY_INFO, io_safety_off ? "Safety switch: off (outputs may run)"
                                                                     : "Safety switch: on (outputs safe)");
            }
            if (io.rc().valid && now_ms >= next_rc_report_ms) {
                next_rc_report_ms = now_ms + 200; // 5 Hz RC_CHANNELS_RAW
                const uint16_t* c = io.rc().pwm;
                const int rssi = io.rc().rssi < 0 ? 255 : (io.rc().rssi > 254 ? 254 : io.rc().rssi);
                mav.send_rc_channels(now_ms, 0, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
                                     static_cast<uint8_t>(rssi));
            }
        }

        const auto reboot = mav.take_reboot_request();
        if (reboot != apx::gcs::MavlinkHandler::RebootRequest::None) {
            if (link != nullptr) {
                link->flush(); // let the COMMAND_ACK reach the ground station
            }
            apx::kernel::delay_ms(100);
            apx::board::reboot(reboot == apx::gcs::MavlinkHandler::RebootRequest::Bootloader);
        }

        if (apx::kernel::now_ms() >= next_notice_ms) {
            mav.send_statustext(MAV_SEVERITY_WARNING,
                                "APX bring-up build: no sensors or motor outputs yet, arming disabled");
            next_notice_ms += kBringupNoticePeriodS * 1000ULL;
        }

        if (++loops % (kLoopHz / 2) == 0) {
            apx::board::toggle_status_led(); // 1 Hz blink: firmware running
        }
    }
    return 0;
}

} // namespace apx::firmware
