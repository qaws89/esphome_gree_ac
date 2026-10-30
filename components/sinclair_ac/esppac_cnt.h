// based on: https://github.com/DomiStyle/esphome-panasonic-ac
#include "esppac_cnt.h"

namespace esphome {
namespace sinclair_ac {
namespace CNT {

static const char *const TAG = "sinclair_ac.serial";

void SinclairACCNT::setup()
{
    SinclairAC::setup();
    ESP_LOGD(TAG, "Using serial protocol for Sinclair AC");
}

void SinclairACCNT::loop()
{
    SinclairAC::loop();
    if (this->serialProcess_.state == STATE_COMPLETE)
    {
        this->serialProcess_.state = STATE_RESTART;
        this->wait_response_ = false;
        log_packet(this->serialProcess_.data);
        if (!verify_packet()) return;
        this->last_packet_received_ = millis();
        if (this->state_ != ACState::Ready)
        {
            this->state_ = ACState::Ready;
            Component::status_clear_error();
            this->last_packet_sent_ = millis();
        }
        if (this->update_ == ACUpdate::NoUpdate) handle_packet();
    }
    send_packet();
    if (millis() - this->last_packet_received_ >= protocol::TIME_TIMEOUT_INACTIVE_MS)
    {
        if (this->state_ != ACState::Initializing)
        {
            this->state_ = ACState::Initializing;
            Component::status_set_error();
        }
    }
}

void SinclairACCNT::control(const climate::ClimateCall &call)
{
    if (this->state_ != ACState::Ready) return;

    if (call.get_mode().has_value())
    {
        this->update_ = ACUpdate::UpdateStart;
        this->mode = *call.get_mode();
    }
    if (call.get_target_temperature().has_value())
    {
        this->update_ = ACUpdate::UpdateStart;
        this->target_temperature = *call.get_target_temperature();
    }
    if (call.has_custom_fan_mode())
    {
        this->update_ = ACUpdate::UpdateStart;
        this->set_custom_fan_mode_(call.get_custom_fan_mode());
    }
    if (call.get_swing_mode().has_value())
    {
        this->update_ = ACUpdate::UpdateStart;
        switch (*call.get_swing_mode()) {
            case climate::CLIMATE_SWING_BOTH:
                this->vertical_swing_state_ = vertical_swing_options::FULL;
                this->horizontal_swing_state_ = horizontal_swing_options::FULL;
                break;
            case climate::CLIMATE_SWING_OFF:
                this->vertical_swing_state_ = vertical_swing_options::CMID;
                this->horizontal_swing_state_ = vertical_swing_options::CMID;
                break;
            case climate::CLIMATE_SWING_VERTICAL:
                this->vertical_swing_state_ = vertical_swing_options::FULL;
                this->horizontal_swing_state_ = vertical_swing_options::CMID;
                break;
            case climate::CLIMATE_SWING_HORIZONTAL:
                this->vertical_swing_state_ = vertical_swing_options::CMID;
                this->horizontal_swing_state_ = vertical_swing_options::FULL;
                break;
            default:
                this->vertical_swing_state_ = vertical_swing_options::CMID;
                this->horizontal_swing_state_ = vertical_swing_options::CMID;
                break;
        }
    }
}

void SinclairACCNT::send_packet()
{
    std::vector<uint8_t> packet(protocol::SET_PACKET_LEN, 0);
    if (this->wait_response_ && (millis() - this->last_packet_sent_) < protocol::TIME_REFRESH_PERIOD_MS) return;

    packet[protocol::SET_CONST_02_BYTE] = protocol::SET_CONST_02_VAL;
    packet[protocol::SET_CONST_BIT_BYTE] = protocol::SET_CONST_BIT_MASK;

    switch(this->update_)
    {
        default:
        case ACUpdate::NoUpdate:
            packet[protocol::SET_NOCHANGE_BYTE] |= protocol::SET_NOCHANGE_MASK;
            break;
        case ACUpdate::UpdateStart:
            packet[protocol::SET_AF_BYTE] = protocol::SET_AF_VAL;
            break;
        case ACUpdate::UpdateClear:
            break;
    }

    uint8_t mode = protocol::REPORT_MODE_AUTO;
    bool power = false;
    switch (this->mode)
    {
        case climate::CLIMATE_MODE_AUTO: mode = protocol::REPORT_MODE_AUTO; power = true; break;
        case climate::CLIMATE_MODE_COOL: mode = protocol::REPORT_MODE_COOL; power = true; break;
        case climate::CLIMATE_MODE_DRY: mode = protocol::REPORT_MODE_DRY; power = true; break;
        case climate::CLIMATE_MODE_FAN_ONLY: mode = protocol::REPORT_MODE_FAN; power = true; break;
        case climate::CLIMATE_MODE_HEAT: mode = protocol::REPORT_MODE_HEAT; power = true; break;
        default:
        case climate::CLIMATE_MODE_OFF:
            switch (this->mode_internal_)
            {
                case climate::CLIMATE_MODE_AUTO: mode = protocol::REPORT_MODE_AUTO; break;
                case climate::CLIMATE_MODE_COOL: mode = protocol::REPORT_MODE_COOL; break;
                case climate::CLIMATE_MODE_DRY: mode = protocol::REPORT_MODE_DRY; break;
                case climate::CLIMATE_MODE_FAN_ONLY: mode = protocol::REPORT_MODE_FAN; break;
                case climate::CLIMATE_MODE_HEAT: mode = protocol::REPORT_MODE_HEAT; break;
            }
            power = false;
            break;
    }
    packet[protocol::REPORT_MODE_BYTE] |= (mode << protocol::REPORT_MODE_POS);
    if (power) packet[protocol::REPORT_PWR_BYTE] |= protocol::REPORT_PWR_MASK;

    /* TARGET TEMPERATURE --------------------------------------------------------------------------- */
    if (this->target_temperature < 16) {
        uint8_t low = (uint8_t)((15 - (uint8_t)this->target_temperature) << 4);
        packet[protocol::REPORT_TEMP_SET_LOW_BYTE] |= low;
        packet[protocol::REPORT_TEMP_SET_BYTE] |= (((16 - protocol::REPORT_TEMP_SET_OFF) << protocol::REPORT_TEMP_SET_POS) & protocol::REPORT_TEMP_SET_MASK);
    } else {
        uint8_t target_temperature = ((((uint8_t)this->target_temperature) - protocol::REPORT_TEMP_SET_OFF) << protocol::REPORT_TEMP_SET_POS);
        packet[protocol::REPORT_TEMP_SET_BYTE] |= (target_temperature & protocol::REPORT_TEMP_SET_MASK);
    }

    /* FAN SPEED --------------------------------------------------------------------------- */
    uint8_t fanSpeed1 = 8, fanSpeed2 = 0;
    bool fanQuiet = false, fanTurbo = false;
    if (this->has_custom_fan_mode())
    {
        const char* m = this->get_custom_fan_mode().c_str();
        if (strcmp(m, fan_modes::FAN_AUTO) == 0)      { fanSpeed1 = 8;  fanSpeed2 = 0; }
        else if (strcmp(m, fan_modes::FAN_LOW) == 0)  { fanSpeed1 = 9;  fanSpeed2 = 1; }
        else if (strcmp(m, fan_modes::FAN_QUIET) == 0){ fanSpeed1 = 9;  fanSpeed2 = 1; fanQuiet = true; }
        else if (strcmp(m, fan_modes::FAN_MEDL) == 0) { fanSpeed1 = 10; fanSpeed2 = 2; }
        else if (strcmp(m, fan_modes::FAN_MED) == 0)  { fanSpeed1 = 11; fanSpeed2 = 2; }
        else if (strcmp(m, fan_modes::FAN_MEDH) == 0) { fanSpeed1 = 12; fanSpeed2 = 3; }
        else if (strcmp(m, fan_modes::FAN_HIGH) == 0) { fanSpeed1 = 13; fanSpeed2 = 3; }
        else if (strcmp(m, fan_modes::FAN_TURBO) == 0){ fanSpeed1 = 8;  fanSpeed2 = 0; fanTurbo = true; }
    }
    packet[protocol::REPORT_FAN_SPD1_BYTE] |= (fanSpeed1 << protocol::REPORT_FAN_SPD1_POS);
    packet[protocol::REPORT_FAN_SPD2_BYTE] |= (fanSpeed2 << protocol::REPORT_FAN_SPD2_POS);
    if (fanTurbo) packet[protocol::REPORT_FAN_TURBO_BYTE] |= protocol::REPORT_FAN_TURBO_MASK;
    if (fanQuiet) packet[protocol::REPORT_FAN_QUIET_BYTE] |= protocol::REPORT_FAN_QUIET_MASK;

    /* VERTICAL SWING */
    uint8_t vs = protocol::REPORT_VSWING_OFF;
    if (this->vertical_swing_state_ == vertical_swing_options::OFF) vs = protocol::REPORT_VSWING_OFF;
    else if (this->vertical_swing_state_ == vertical_swing_options::FULL) vs = protocol::REPORT_VSWING_FULL;
    else if (this->vertical_swing_state_ == vertical_swing_options::DOWN) vs = protocol::REPORT_VSWING_DOWN;
    else if (this->vertical_swing_state_ == vertical_swing_options::MIDD) vs = protocol::REPORT_VSWING_MIDD;
    else if (this->vertical_swing_state_ == vertical_swing_options::MID) vs = protocol::REPORT_VSWING_MID;
    else if (this->vertical_swing_state_ == vertical_swing_options::MIDU) vs = protocol::REPORT_VSWING_MIDU;
    else if (this->vertical_swing_state_ == vertical_swing_options::UP) vs = protocol::REPORT_VSWING_UP;
    else if (this->vertical_swing_state_ == vertical_swing_options::CDOWN) vs = protocol::REPORT_VSWING_CDOWN;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMIDD) vs = protocol::REPORT_VSWING_CMIDD;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMID) vs = protocol::REPORT_VSWING_CMID;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMIDU) vs = protocol::REPORT_VSWING_CMIDU;
    else if (this->vertical_swing_state_ == vertical_swing_options::CUP) vs = protocol::REPORT_VSWING_CUP;
    packet[protocol::REPORT_VSWING_BYTE] |= (vs << protocol::REPORT_VSWING_POS);

    /* HORIZONTAL SWING */
    uint8_t hs = protocol::REPORT_HSWING_OFF;
    if (this->horizontal_swing_state_ == horizontal_swing_options::OFF) hs = protocol::REPORT_HSWING_OFF;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::FULL) hs = protocol::REPORT_HSWING_FULL;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CLEFT) hs = protocol::REPORT_HSWING_CLEFT;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CMIDL) hs = protocol::REPORT_HSWING_CMIDL;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CMID) hs = protocol::REPORT_HSWING_CMID;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CMIDR) hs = protocol::REPORT_HSWING_CMIDR;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CRIGHT) hs = protocol::REPORT_HSWING_CRIGHT;
    packet[protocol::REPORT_HSWING_BYTE] |= (hs << protocol::REPORT_HSWING_POS);

    /* DISPLAY */
    uint8_t dm = protocol::REPORT_DISP_MODE_AUTO;
    if (this->display_state_ == display_options::AUTO) { dm = protocol::REPORT_DISP_MODE_AUTO; this->display_power_internal_ = true; }
    else if (this->display_state_ == display_options::SET) { dm = protocol::REPORT_DISP_MODE_SET; this->display_power_internal_ = true; }
    else if (this->display_state_ == display_options::ACT) { dm = protocol::REPORT_DISP_MODE_ACT; this->display_power_internal_ = true; }
    else if (this->display_state_ == display_options::OUT) { dm = protocol::REPORT_DISP_MODE_OUT; this->display_power_internal_ = true; }
    else if (this->display_state_ == display_options::OFF) {
        this->display_power_internal_ = false;
        if (this->display_mode_internal_ == display_options::SET) dm = protocol::REPORT_DISP_MODE_SET;
        else if (this->display_mode_internal_ == display_options::ACT) dm = protocol::REPORT_DISP_MODE_ACT;
        else if (this->display_mode_internal_ == display_options::OUT) dm = protocol::REPORT_DISP_MODE_OUT;
        else dm = protocol::REPORT_DISP_MODE_AUTO;
    } else { dm = protocol::REPORT_DISP_MODE_AUTO; this->display_power_internal_ = true; }
    packet[protocol::REPORT_DISP_MODE_BYTE] |= (dm << protocol::REPORT_DISP_MODE_POS);
    if (this->display_power_internal_) packet[protocol::REPORT_DISP_ON_BYTE] |= protocol::REPORT_DISP_ON_MASK;

    if (this->display_unit_state_ == display_unit_options::DEGF) packet[protocol::REPORT_DISP_F_BYTE] |= protocol::REPORT_DISP_F_MASK;
    if (this->plasma_state_) { packet[protocol::REPORT_PLASMA1_BYTE] |= protocol::REPORT_PLASMA1_MASK; packet[protocol::REPORT_PLASMA2_BYTE] |= protocol::REPORT_PLASMA2_MASK; }
    if (this->sleep_state_) packet[protocol::REPORT_SLEEP_BYTE] |= protocol::REPORT_SLEEP_MASK;
    if (this->xfan_state_) packet[protocol::REPORT_XFAN_BYTE] |= protocol::REPORT_XFAN_MASK;
    if (this->save_state_) packet[protocol::REPORT_SAVE_BYTE] |= protocol::REPORT_SAVE_MASK;

    packet.insert(packet.begin(), protocol::CMD_OUT_PARAMS_SET);
    packet.insert(packet.begin(), protocol::SET_PACKET_LEN + 2);
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < packet.size(); i++) checksum += packet[i];
    packet.push_back(checksum);
    packet.insert(packet.begin(), protocol::SYNC);
    packet.insert(packet.begin(), protocol::SYNC);

    this->last_packet_sent_ = millis();
    this->wait_response_ = true;
    write_array(packet);
    log_packet(packet, true);

    switch(this->update_)
    {
        case ACUpdate::NoUpdate: break;
        case ACUpdate::UpdateStart: this->update_ = ACUpdate::UpdateClear; break;
        case ACUpdate::UpdateClear: this->update_ = ACUpdate::NoUpdate; break;
        default: this->update_ = ACUpdate::NoUpdate; break;
    }
}
