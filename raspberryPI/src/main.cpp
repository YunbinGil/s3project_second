// main.cpp — Raspberry Pi UART 통신 대화형 테스트
//
// 명령어:
//   PING                        → 전체 노드 PING
//   PING N                      → Node N만 PING
//   RANGE                       → 전체 노드 거리 측정
//   RANGE N                     → Node N만 거리 측정
//   CMD <topic> <payload>       → 전체 노드 CTRL_FWD
//   CMD N <topic> <payload>     → Node N만 CTRL_FWD  (N = 1/2/3)
//   quit
//
//  예)
//   CMD 1 command/light C9      → Node1만 emergency
//   CMD 2 command/light P1      → Node2만 ped extend
//   CMD command/light C9        → 전체 emergency

#include "uart_manager.h"
#include "protocol.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <cmath>
#include <array>

#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <time.h>

static volatile sig_atomic_t g_running = 1;
static void on_signal(int) { g_running = 0; }

struct NodeRange {
    bool     valid           = false;
    uint16_t distance_cm     = 0;
    int16_t  rssi_centi_dbm  = 0;
    uint64_t updated_ms      = 0;
};

static NodeRange g_ranges[4];
static uint64_t  g_now_ms = 0;

static bool compute_position(const std::array<double,3>& dist_m,
                              double* out_x, double* out_y) {
    const double x1=0.0,y1=0.0, x2=2.0,y2=0.0, x3=1.0,y3=1.732;
    const double d1=dist_m[0], d2=dist_m[1], d3=dist_m[2];
    const double A=2*(x2-x1), B=2*(y2-y1);
    const double C=(d1*d1-d2*d2)-(x1*x1-x2*x2)-(y1*y1-y2*y2);
    const double D=2*(x3-x1), E=2*(y3-y1);
    const double F=(d1*d1-d3*d3)-(x1*x1-x3*x3)-(y1*y1-y3*y3);
    const double det=A*E-B*D;
    if (std::fabs(det)<1e-9) return false;
    *out_x=(C*E-B*F)/det;
    *out_y=(A*F-C*D)/det;
    return true;
}

static void try_print_position() {
    const uint64_t freshness_ms = 1500;
    for (int n=1;n<=3;n++) {
        if (!g_ranges[n].valid) return;
        if (g_now_ms - g_ranges[n].updated_ms > freshness_ms) return;
    }
    std::array<double,3> dist_m = {
        g_ranges[1].distance_cm/100.0,
        g_ranges[2].distance_cm/100.0,
        g_ranges[3].distance_cm/100.0,
    };
    double x=0,y=0;
    if (!compute_position(dist_m,&x,&y)) {
        fprintf(stderr,"[POS] trilateration failed\n"); return;
    }
    printf("[POS] car=(%.3f,%.3f)m  d=[%.2f,%.2f,%.2f]m\n",
           x,y,dist_m[0],dist_m[1],dist_m[2]);
}

static void on_pong(int node_id, const uint8_t* payload, uint8_t len) {
    if (len<1) return;
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC,&ts);
    g_now_ms = (uint64_t)ts.tv_sec*1000ULL + ts.tv_nsec/1000000ULL;

    if (payload[0]==proto::CMD_PONG && len>=4) {
        uint16_t seq=((uint16_t)payload[1]<<8)|payload[2];
        printf("[RX] PONG <- Node%d  NodeID=%u  SEQ=%u\n", node_id, payload[3], seq);
    } else if (payload[0]==proto::CMD_RANGE_REPORT && len>=7) {
        uint8_t  rn   = payload[1];
        uint8_t  car  = payload[2];
        uint16_t dcm  = ((uint16_t)payload[3]<<8)|payload[4];
        int16_t  rssi = (int16_t)(((uint16_t)payload[5]<<8)|payload[6]);
        if (rn<1||rn>3) { printf("[RX] RANGE_REPORT invalid node=%u\n",rn); return; }
        if (dcm==0xFFFFu) {
            printf("[RX] RANGE_REPORT Node%u car=%u FAILED\n",rn,car);
            g_ranges[rn].valid=false; return;
        }
        g_ranges[rn]={true,dcm,rssi,g_now_ms};
        printf("[RX] RANGE_REPORT Node%u  car=%u  dist=%ucm  rssi=%.2fdBm\n",
               rn,car,dcm,rssi/100.0);
        try_print_position();
    } else if (payload[0]==proto::CMD_MODE_NOTIFY && len>=3) {
        printf("[RX] MODE_NOTIFY <- Node%d  %s\n", node_id,
               payload[2]==1 ? "MQTT_CONTROL" : "HTTP_CONTROL");
    } else {
        printf("[RX] Node%d: cmd=0x%02X len=%u\n", node_id, payload[0], len);
    }
}

// ── PING 전송 ─────────────────────────────────
static void send_ping(UartManager& mgr, int node, uint16_t* seq) {
    uint8_t p[3];
    p[0]=proto::CMD_PING;
    p[1]=(*seq>>8)&0xFF;
    p[2]=*seq&0xFF;
    if (mgr.send(node,p,3)) printf("[TX] PING -> Node%d  SEQ=%u\n",node,*seq);
    (*seq)++;
}

static void send_range(UartManager& mgr, int node, uint16_t* seq) {
    uint8_t p[3];
    p[0]=proto::CMD_RANGE_REQUEST;
    p[1]=(*seq>>8)&0xFF;
    p[2]=*seq&0xFF;
    if (mgr.send(node,p,3)) printf("[TX] RANGE_REQUEST -> Node%d  SEQ=%u\n",node,*seq);
    (*seq)++;
}

// ── CMD_CTRL_FWD 전송 ─────────────────────────
static void send_ctrl_fwd(UartManager& mgr, int node,
                           const char* topic, const char* payload_str) {
    size_t tlen = strlen(topic);
    size_t plen = strlen(payload_str);
    if (tlen>63||plen>63) { printf("[!] topic/payload too long\n"); return; }

    uint8_t buf[132];
    size_t pos=0;
    buf[pos++]=proto::CMD_CTRL_FWD;
    buf[pos++]=(uint8_t)tlen;
    memcpy(buf+pos,topic,tlen);       pos+=tlen;
    buf[pos++]=(plen>>8)&0xFF;
    buf[pos++]=plen&0xFF;
    memcpy(buf+pos,payload_str,plen); pos+=plen;

    if (mgr.send(node,buf,(uint8_t)pos))
        printf("[TX] CMD -> Node%d  topic='%s'  payload='%s'\n",node,topic,payload_str);
}

// ── main ──────────────────────────────────────
int main(int argc, char* argv[]) {
    signal(SIGINT,on_signal);
    signal(SIGTERM,on_signal);

    const char* default_dev[3]={
        "/dev/ttyESP_Node1","/dev/ttyESP_Node2","/dev/ttyESP_Node3"
    };
    const int active_count=(argc>1)?(argc-1):3;
    if (active_count<1||active_count>3) {
        fprintf(stderr,"usage: %s [dev1] [dev2] [dev3]\n",argv[0]); return 1;
    }
    const char* dev[3]={
        (argc>1)?argv[1]:default_dev[0],
        (argc>2)?argv[2]:default_dev[1],
        (argc>3)?argv[3]:default_dev[2],
    };

    UartManager mgr;
    mgr.set_rx_callback(on_pong);
    for (int i=0;i<active_count;i++) {
        if (!mgr.open_port(i+1,dev[i],115200)) return 1;
    }

    // stdin non-blocking
    int flags=fcntl(STDIN_FILENO,F_GETFL,0);
    fcntl(STDIN_FILENO,F_SETFL,flags|O_NONBLOCK);
    struct epoll_event ev{};
    ev.events=EPOLLIN; ev.data.fd=STDIN_FILENO;
    if (epoll_ctl(mgr.epoll_fd(),EPOLL_CTL_ADD,STDIN_FILENO,&ev)<0) {
        perror("epoll_ctl stdin"); return 1;
    }

    printf("=== UART 테스트 (%d개 노드) ===\n",active_count);
    printf("PING [N] | RANGE [N] | CMD [N] <topic> <payload> | quit\n");
    printf("  예) CMD 1 command/light C9    (Node1만 emergency)\n");
    printf("      CMD command/light P1      (전체 ped extend)\n");
    printf("      CMD 2 command/light C0    (Node2 차량 RED)\n");
    printf("> "); fflush(stdout);

    uint16_t ping_seq=0, range_seq=0;
    char buf[256]; int bufpos=0;
    struct epoll_event events[8];

    while (g_running) {
        int nfds=epoll_wait(mgr.epoll_fd(),events,8,500);
        if (nfds<0) { if(errno==EINTR)continue; perror("epoll_wait"); break; }

        for (int i=0;i<nfds;i++) {
            int fd=events[i].data.fd;
            if (fd==STDIN_FILENO) {
                char ch;
                while (read(STDIN_FILENO,&ch,1)==1) {
                    if (ch=='\n') {
                        buf[bufpos]='\0';
                        struct timespec ts{};
                        clock_gettime(CLOCK_MONOTONIC,&ts);
                        g_now_ms=(uint64_t)ts.tv_sec*1000ULL+ts.tv_nsec/1000000ULL;

                        // ── 명령 파싱 ──────────────────────────
                        char cmd[256];
                        strncpy(cmd,buf,255); cmd[255]='\0';

                        if (strcasecmp(cmd,"PING")==0) {
                            for(int n=1;n<=active_count;n++) send_ping(mgr,n,&ping_seq);

                        } else if (strncasecmp(cmd,"PING ",5)==0) {
                            int n=atoi(cmd+5);
                            if(n>=1&&n<=active_count) send_ping(mgr,n,&ping_seq);
                            else printf("[!] Node 번호 1~%d\n",active_count);

                        } else if (strcasecmp(cmd,"RANGE")==0) {
                            for(int n=1;n<=active_count;n++) send_range(mgr,n,&range_seq);

                        } else if (strncasecmp(cmd,"RANGE ",6)==0) {
                            int n=atoi(cmd+6);
                            if(n>=1&&n<=active_count) send_range(mgr,n,&range_seq);
                            else printf("[!] Node 번호 1~%d\n",active_count);

                        } else if (strncasecmp(cmd,"CMD ",4)==0) {
                            char* rest=cmd+4;

                            // 숫자로 시작하면 → CMD N topic payload
                            int target_node = 0;
                            if (rest[0]>='1' && rest[0]<='3' && rest[1]==' ') {
                                target_node = rest[0]-'0';
                                rest += 2;
                            }

                            char* sp=strchr(rest,' ');
                            if (!sp) {
                                printf("[!] 형식: CMD [N] <topic> <payload>\n");
                            } else {
                                *sp='\0';
                                const char* topic  = rest;
                                const char* payload= sp+1;

                                if (target_node>0) {
                                    // 특정 노드만
                                    if(target_node<=active_count)
                                        send_ctrl_fwd(mgr,target_node,topic,payload);
                                    else
                                        printf("[!] Node%d 없음\n",target_node);
                                } else {
                                    // 전체
                                    for(int n=1;n<=active_count;n++)
                                        send_ctrl_fwd(mgr,n,topic,payload);
                                }
                            }

                        } else if (strcasecmp(cmd,"quit")==0||strcasecmp(cmd,"exit")==0) {
                            g_running=0;
                        } else if (bufpos>0) {
                            printf("[?] '%s'\n",cmd);
                        }

                        bufpos=0;
                        if(g_running){printf("> ");fflush(stdout);}
                    } else if(bufpos<(int)sizeof(buf)-1){
                        buf[bufpos++]=ch;
                    }
                }
            } else {
                if(events[i].events&EPOLLIN) mgr.dispatch(fd);
            }
        }
    }
    printf("\n=== 종료 ===\n");
    return 0;
}
