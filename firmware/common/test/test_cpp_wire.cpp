// test_cpp_wire.cpp — C++ golden-fixture round trip for robot_wire_gen.hpp.
//
// For each fixture in generated/cpp/fixtures/ it decodes the bytes, checks the
// known values (the same ones gen_fixture.c packs), re-encodes, and
// byte-compares against the file. Exits non-zero on any mismatch.
//
// Build:  g++ -std=c++17 -Wall -Wextra -Werror -I generated/cpp
//             firmware/common/test/test_cpp_wire.cpp -o test_cpp_wire
// Run:    ./test_cpp_wire generated/cpp/fixtures
#include "robot_wire_gen.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace robot_wire;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

static std::vector<std::uint8_t> read_file(const std::string &path)
{
    std::vector<std::uint8_t> out;
    FILE *f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) {
        std::printf("FAIL: cannot open %s\n", path.c_str());
        ++failures;
        return out;
    }
    std::uint8_t buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) {
        out.insert(out.end(), buf, buf + n);
    }
    std::fclose(f);
    return out;
}

template <typename T>
static void round_trip(const std::vector<std::uint8_t> &raw, const T &decoded)
{
    const auto re = to_bytes(decoded);
    CHECK(re.size() == raw.size());
    CHECK(std::memcmp(re.data(), raw.data(), raw.size()) == 0);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::printf("usage: %s <fixtures-dir>\n", argv[0]);
        return 2;
    }
    const std::string dir = argv[1];

    // ── cmd_robot_t fixture ──
    {
        const auto raw = read_file(dir + "/robot_cmd.bin");
        CHECK(raw.size() == sizeof(cmd_robot_t));
        if (raw.size() == sizeof(cmd_robot_t)) {
            const auto c = from_bytes<cmd_robot_t>(raw.data());
            CHECK(c.cycle_id == 7u);
            CHECK(c.cmd_seq == 42u);
            CHECK(c.n_chains == 1u);
            CHECK(c.reserved == 0xDEu);          // master test sentinel, pinned
            const cmd_motor_t m = c.chains[0].motors[0];  // copy, no packed ref
            CHECK(m.drive_req ==
                  static_cast<std::uint8_t>(DriveReq::DRV_REQ_ENABLE));
            CHECK(m.pos == 5000);
            CHECK(m.vel == -125);
            CHECK(m.kp == 150u);
            CHECK(m.kd == 100u);
            CHECK(m.tau_ff == 30);
            CHECK(m.flags == CMD_FLAG_VALID);
            round_trip(raw, c);
        }
    }

    // ── tele_robot_t fixture ──
    {
        const auto raw = read_file(dir + "/robot_tele.bin");
        CHECK(raw.size() == sizeof(tele_robot_t));
        if (raw.size() == sizeof(tele_robot_t)) {
            const auto t = from_bytes<tele_robot_t>(raw.data());
            CHECK(t.cycle_id == 9u);
            CHECK(t.master_time_us == 123456u);
            CHECK(t.last_cmd_seq_rx == 42u);
            CHECK(t.cmd_seq_active == 41u);
            CHECK(t.cmd_on_time == 1000u);
            CHECK(t.cmd_late == 7u);
            CHECK(t.cmd_missing == 3u);
            CHECK(t.cmd_duplicate == 2u);
            CHECK(t.n_chains == 1u);
            CHECK(t.robot_state ==
                  static_cast<std::uint8_t>(RobotState::ROBOT_READY));
            const tele_chain_t ch = t.chains[0];
            CHECK(ch.link ==
                  static_cast<std::uint8_t>(ChainLink::CHAIN_LINK_ALIVE));
            CHECK(ch.spi_seq_echo == 0x2Au);
            CHECK(ch.slave_time_us == 7777u);
            CHECK(ch.cmd_crc_errors == 3u);
            CHECK(ch.can_tx_errors == 1u);
            CHECK(ch.spi_tx_arm_fails == 4u);
            const tele_motor_t m = ch.motors[0];
            CHECK(m.pos == 5000);
            CHECK(m.vel == -125);
            CHECK(m.tau == 30);
            CHECK(m.temp_c == 42u);
            CHECK(m.drive_state ==
                  static_cast<std::uint8_t>(DriveState::DRV_ENABLED));
            CHECK(m.drive_fault ==
                  static_cast<std::uint8_t>(DriveFault::DRVF_NONE));
            CHECK(m.rs_mode == 2u);
            CHECK(m.rs_fault_bits == 0x0Au);
            CHECK(m.flags == TELE_FLAG_SATURATED);
            CHECK(m.fb_age_ms == 250u);
            CHECK(m.rs_fault_word == 0xDEADBEEFu);
            CHECK(m.last_applied_seq == 0x1234u);
            round_trip(raw, t);
        }
    }

    // A couple of constants the firmware side also pins.
    CHECK(PROTO_VERSION == 10u);
    CHECK(MSG_HEADER_SIZE == 16u);
    CHECK(PROTO_MAX_PAYLOAD == sizeof(tele_robot_t));
    CHECK(SPI_XFER_SIZE == (SPI_CMD_FRAME_SIZE > SPI_TELE_FRAME_SIZE
                            ? SPI_CMD_FRAME_SIZE : SPI_TELE_FRAME_SIZE));

    if (failures == 0) {
        std::printf("test_cpp_wire: OK\n");
        return 0;
    }
    std::printf("test_cpp_wire: %d failure(s)\n", failures);
    return 1;
}
