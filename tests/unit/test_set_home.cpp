// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
//
// MAV_CMD_DO_SET_HOME through the real MAVLink path: packets in, COMMAND_ACK
// and the home that RTL flies to (home_ned) out.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "mavlink_handler.hpp"

using apx::gcs::MavlinkHandler;

namespace {

constexpr double kBootLat = 18.5204;
constexpr double kBootLon = 73.8567;
constexpr float kBootAlt = 540.0f;

class SetHomeTest : public ::testing::Test {
protected:
    void SetUp() override {
        m_mav.init();
        m_mav.set_byte_writer(
            [](const uint8_t* data, size_t len, void* ctx) {
                auto* out = static_cast<std::vector<uint8_t>*>(ctx);
                out->insert(out->end(), data, data + len);
            },
            &m_out);
        m_mav.set_home_position(kBootLat, kBootLon, kBootAlt); // boot home = NED origin
    }

    // Returns the COMMAND_ACK result for [command], or -1 if none was sent.
    int last_ack(uint16_t command) {
        mavlink_message_t msg;
        mavlink_status_t status{};
        int result = -1;
        for (uint8_t b : m_out) {
            if (mavlink_parse_char(MAVLINK_COMM_3, b, &msg, &status) && msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);
                if (ack.command == command) result = ack.result;
            }
        }
        m_out.clear();
        return result;
    }

    void send(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        m_mav.receive_bytes(buf, len);
    }

    int set_home_long(float use_current, float lat, float lon, float alt) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(255, 190, &msg, 1, 1, MAV_CMD_DO_SET_HOME, 0, use_current, 0, 0, 0, lat, lon,
                                      alt);
        send(msg);
        return last_ack(MAV_CMD_DO_SET_HOME);
    }

    int set_home_int(double lat, double lon, float alt) {
        mavlink_message_t msg;
        mavlink_msg_command_int_pack(255, 190, &msg, 1, 1, MAV_FRAME_GLOBAL, MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0,
                                     static_cast<int32_t>(std::lround(lat * 1e7)),
                                     static_cast<int32_t>(std::lround(lon * 1e7)), alt);
        send(msg);
        return last_ack(MAV_CMD_DO_SET_HOME);
    }

    MavlinkHandler m_mav{1, 1};
    std::vector<uint8_t> m_out;
};

// lat/lon of a point [north]/[east] metres from the boot home.
void offset(double north, double east, double& lat, double& lon) {
    constexpr double kR = 6378137.0;
    lat = kBootLat + north / kR * 180.0 / M_PI;
    lon = kBootLon + east / (kR * std::cos(kBootLat * M_PI / 180.0)) * 180.0 / M_PI;
}

} // namespace

TEST_F(SetHomeTest, CommandIntMovesTheReturnPointExactly) {
    double lat, lon;
    offset(30.0, 50.0, lat, lon);
    EXPECT_EQ(set_home_int(lat, lon, 545.0f), MAV_RESULT_ACCEPTED);
    EXPECT_NEAR(m_mav.home_lat(), lat, 1e-7);
    EXPECT_NEAR(m_mav.home_lon(), lon, 1e-7);
    EXPECT_FLOAT_EQ(m_mav.home_alt_msl(), 545.0f);
    EXPECT_TRUE(m_mav.is_home_locked()); // arming / landing must not move it back

    float ned[3];
    m_mav.home_ned(ned); // what RTL flies to
    EXPECT_NEAR(ned[0], 30.0f, 0.05f);
    EXPECT_NEAR(ned[1], 50.0f, 0.05f);
}

TEST_F(SetHomeTest, CommandLongWorksAndUnknownAltitudeKeepsTheOldOne) {
    double lat, lon;
    offset(-20.0, 10.0, lat, lon);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(set_home_long(0, static_cast<float>(lat), static_cast<float>(lon), nan), MAV_RESULT_ACCEPTED);
    EXPECT_FLOAT_EQ(m_mav.home_alt_msl(), kBootAlt);
    float ned[3];
    m_mav.home_ned(ned);
    EXPECT_NEAR(ned[0], -20.0f, 1.0f); // float params: ~1 m resolution at this longitude
    EXPECT_NEAR(ned[1], 10.0f, 1.0f);
}

TEST_F(SetHomeTest, UseCurrentPositionTakesTheVehiclePosition) {
    double lat, lon;
    offset(5.0, -7.0, lat, lon);
    m_mav.set_vehicle_state(0, 0, 0, lat, lon, 552.0f, 12.0f, 0, 0, 0, 12.6f, 2.5f, 98, false,
                            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6);
    EXPECT_EQ(set_home_long(1, 0, 0, 0), MAV_RESULT_ACCEPTED);
    EXPECT_NEAR(m_mav.home_lat(), lat, 1e-9);
    EXPECT_NEAR(m_mav.home_lon(), lon, 1e-9);
    EXPECT_FLOAT_EQ(m_mav.home_alt_msl(), 552.0f);
}

TEST_F(SetHomeTest, InvalidPositionsAreRefusedAndHomeStays) {
    EXPECT_EQ(set_home_int(95.0, 73.0, 540.0f), MAV_RESULT_DENIED);
    EXPECT_EQ(set_home_int(18.0, 181.0, 540.0f), MAV_RESULT_DENIED);
    EXPECT_EQ(set_home_long(0, 0, 0, 540.0f), MAV_RESULT_DENIED); // (0, 0): an unset position
    EXPECT_EQ(set_home_long(0, std::numeric_limits<float>::quiet_NaN(), 73.0f, 540.0f), MAV_RESULT_DENIED);
    EXPECT_DOUBLE_EQ(m_mav.home_lat(), kBootLat);
    EXPECT_DOUBLE_EQ(m_mav.home_lon(), kBootLon);
    EXPECT_FALSE(m_mav.is_home_locked());
}
