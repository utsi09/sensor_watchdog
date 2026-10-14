#pragma once
// ROS 2 직렬화 메시지(CDR)에서 header.stamp만 읽는다.
// 전제: 메시지의 첫 필드가 std_msgs/Header 이어야 한다.
// sensor_msgs의 PointCloud2, Image, CompressedImage, NavSatFix, Imu 모두 해당.
//
// CDR 인캡슐레이션 4바이트: [0]=0x00, [1]=0x00(BE) 또는 0x01(LE), [2..3]=옵션
// 그 뒤 int32 sec, uint32 nanosec, string frame_id 순서.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace sensor_watchdog
{

struct Stamp
{
    int32_t sec;
    uint32_t nanosec;
};

inline std::optional<Stamp> read_header_stamp(const uint8_t *data, std::size_t size)
{
    if (!data || size < 12 || data[0] != 0x00 || data[1] > 0x01)
        return std::nullopt;

    const bool little = (data[1] == 0x01);

    const auto u32 = [&](std::size_t off) {
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            const std::size_t idx = little ? off + i : off + 3 - i;
            v |= static_cast<uint32_t>(data[idx]) << (8 * i);
        }
        return v;
    };

    Stamp s;
    s.sec = static_cast<int32_t>(u32(4));
    s.nanosec = u32(8);
    return s;
}

}  // namespace sensor_watchdog

namespace sensor_watchdog
{

// 게이트 메시지에서 첫 정수 필드 앞에 무엇이 오는가.
//   NONE   : 바로 정수 (std_msgs/Bool, UInt8 등)
//   STAMP  : builtin_interfaces/Time stamp 뒤 정수 (autoware_vehicle_msgs/GearReport 등)
//   HEADER : std_msgs/Header 뒤 정수 (stamp + frame_id 문자열)
enum class Prefix
{
    NONE,
    STAMP,
    HEADER
};

inline bool parse_prefix(const std::string &name, Prefix &out)
{
    if (name == "none")   { out = Prefix::NONE;   return true; }
    if (name == "stamp")  { out = Prefix::STAMP;  return true; }
    if (name == "header") { out = Prefix::HEADER; return true; }
    return false;
}

// 메시지의 첫 정수 필드를 읽는다. 앞의 prefix(없음/stamp/Header)는 건너뛴다.
// 게이트 토픽(기어 상태 등)을 타입 의존 없이 읽기 위한 용도.
//   bytes: 필드 크기. bool/int8/uint8=1, int16=2, int32=4, int64=8
// CDR 규칙: 기본형은 본문 시작(인캡슐레이션 4바이트 뒤) 기준으로 자기 크기에 맞춰 정렬된다.
inline std::optional<uint64_t> read_first_int(
    const uint8_t *data, std::size_t size, Prefix prefix, std::size_t bytes)
{
    if (!data || size < 4 || data[0] != 0x00 || data[1] > 0x01)
        return std::nullopt;
    if (bytes != 1 && bytes != 2 && bytes != 4 && bytes != 8)
        return std::nullopt;

    const bool little = (data[1] == 0x01);
    std::size_t off = 0;   // 본문 기준 오프셋

    const auto u32_at = [&](std::size_t o) -> std::optional<uint32_t> {
        if (4 + o + 4 > size) return std::nullopt;
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            const std::size_t idx = little ? 4 + o + i : 4 + o + 3 - i;
            v |= static_cast<uint32_t>(data[idx]) << (8 * i);
        }
        return v;
    };

    if (prefix == Prefix::STAMP || prefix == Prefix::HEADER)
        off = 8;                               // sec(4) + nanosec(4)

    if (prefix == Prefix::HEADER) {
        const auto len = u32_at(off);          // frame_id 길이(널 포함)
        if (!len) return std::nullopt;
        off += 4 + *len;
    }

    // 필드 크기에 맞춰 정렬
    if (off % bytes != 0)
        off += bytes - (off % bytes);

    if (4 + off + bytes > size)
        return std::nullopt;

    uint64_t v = 0;
    for (std::size_t i = 0; i < bytes; ++i) {
        const std::size_t idx = little ? 4 + off + i : 4 + off + bytes - 1 - i;
        v |= static_cast<uint64_t>(data[idx]) << (8 * i);
    }
    return v;
}

// 이전 인터페이스 호환: has_header=true 는 HEADER, false 는 NONE.
inline std::optional<uint64_t> read_first_int(
    const uint8_t *data, std::size_t size, bool has_header, std::size_t bytes)
{
    return read_first_int(data, size, has_header ? Prefix::HEADER : Prefix::NONE, bytes);
}

}  // namespace sensor_watchdog
