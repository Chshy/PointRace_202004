#pragma once
#include <iostream>
#include "Serial.hpp"

struct ACFly_SDI_Pack_Typedef
{
    const uint8_t head[2] = {'A', 'C'};
    uint8_t type;
    uint8_t len = 0;
    uint8_t body[4 * 6] = {0};
    uint8_t sumA = 0;
    uint8_t sumB = 0;
};

void send_pack(const ACFly_SDI_Pack_Typedef *msg_p)
{
    pi_uart_write((char*)(msg_p->head), 2);
    pi_uart_write((char*)&(msg_p->type), 1);
    pi_uart_write((char*)&(msg_p->len), 1);
    pi_uart_write((char*)(msg_p->body), msg_p->len);
    pi_uart_write((char*)&(msg_p->sumA), 1);
    pi_uart_write((char*)&(msg_p->sumB), 1);
    return;
}

//将给定float分割为4x8bit,存储在buf~buf+3的地址内
void vec2pack(uint8_t msg_type, float src_x, float src_y)
{
    ACFly_SDI_Pack_Typedef msg;

    msg.type = msg_type;
    msg.sumA += msg_type;
    msg.sumB += msg.sumA;

    msg.len = 4 * 2;
    msg.sumA += msg.len;
    msg.sumB += msg.sumA;

    *(float *)(msg.body) = src_x;
    *(float *)(msg.body + 4) = src_y;
    for (int i = 0; i < 8; i++)
    {
        msg.sumA += *(msg.body + i);
        msg.sumB += msg.sumA;
    }

    send_pack(&msg);
    return;
}