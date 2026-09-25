#include "uav/Config.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace uav {
namespace {

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::vector<std::string> splitCommas(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream        ss(s);
    std::string              item;
    while (std::getline(ss, item, ',')) {
        const std::string t = trim(item);
        if (!t.empty()) out.push_back(t);
    }
    return out;
}

} // namespace

bool Config::load(const std::string& path, Config& out, std::string& err) {
    std::ifstream in(path);
    if (!in) { err = "не вдалося відкрити " + path; return false; }

    std::string line;
    int         lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            err = "рядок " + std::to_string(lineNo) + ": немає знака '='";
            return false;
        }
        const std::string key = trim(line.substr(0, eq));
        const std::string val = trim(line.substr(eq + 1));

        auto num  = [&] { return std::strtod(val.c_str(), nullptr); };
        auto inum = [&] { return static_cast<int>(std::strtol(val.c_str(), nullptr, 10)); };
        auto port = [&] { return static_cast<std::uint16_t>(inum()); };

        if      (key == "fc_device")        out.fcDevice    = val;
        else if (key == "fc_baud")          out.fcBaud      = inum();
        else if (key == "radio_device")     out.radioDevice = val;
        else if (key == "radio_baud")       out.radioBaud   = inum();
        else if (key == "udp_port")         out.udpPort     = port();
        else if (key == "peer_host")        out.peerHost    = val;
        else if (key == "peer_port")        out.peerPort    = port();
        else if (key == "echo_port")        out.echoPort    = port();
        else if (key == "control_port")     out.controlPort = port();
        else if (key == "operator_host")    out.operatorHost = val;
        else if (key == "control_allow")    out.controlAllow = splitCommas(val);
        else if (key == "peer_fixed")       out.peerFixed   = (val == "1" || val == "true");
        else if (key == "t_lost_sec")       out.tLostSec    = num();
        else if (key == "t_back_sec")       out.tBackSec    = num();
        else if (key == "rtt_switch_full_ms") out.rttSwitchFullMs = num();
        else if (key == "rtt_back_full_ms")   out.rttBackFullMs   = num();
        else if (key == "rtt_offset_ms")      out.rttOffsetMs     = num();
        else if (key == "rtt_stale_sec")      out.rttStaleSec     = num();
        else if (key == "failsafe_sec")       out.failsafeSec     = num();
        else if (key == "failsafe_action")    out.failsafeAction  = val;
        else if (key == "require_nav_for_rtl")
            out.requireNavForRtl = (val == "1" || val == "true");
        else if (key == "nav_stale_sec")      out.navStaleSec     = num();
        else if (key == "flush_sec")          out.flushSec        = num();
        else if (key == "max_datagram")       out.maxDatagram     = static_cast<std::size_t>(inum());
        else if (key == "max_backlog")        out.maxBacklog      = static_cast<std::size_t>(inum());
        else if (key == "keepalive_sec")      out.keepaliveSec    = num();
        else if (key == "probe_sec")          out.probeSec        = num();
        else if (key == "full_probe_sec")     out.fullProbeSec    = num();
        else if (key == "report_sec")         out.reportSec       = num();
        else {
            err = "рядок " + std::to_string(lineNo) + ": невідомий ключ '" + key + "'";
            return false;
        }
    }

    if (out.failsafeAction != "log" && out.failsafeAction != "rtl") {
        err = "failsafe_action має бути log або rtl";
        return false;
    }
    if (out.rttBackFullMs >= out.rttSwitchFullMs) {
        err = "rtt_back_full_ms має бути меншим за rtt_switch_full_ms (потрібен гістерезис)";
        return false;
    }
    if (out.navStaleSec <= 0.0) {
        err = "nav_stale_sec має бути додатним";
        return false;
    }
    return true;
}

} // namespace uav
