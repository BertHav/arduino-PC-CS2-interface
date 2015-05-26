/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <info@gerhard-bertelsmann.de> wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return
 * Gerhard Bertelsmann
 * ----------------------------------------------------------------------------
 */

/* Thanks to Stefan Krauss and the SocketCAN team
 */

#include "lan2lan.h"

char *CAN_FORMAT_STRG      = "      CAN->  CANID 0x%08X R [%d]";
char *TO_CAN_FORMAT_STRG   = "      CAN    CANID 0x%08X   [%d]";
char *UDP_FORMAT_STRG      = "->CAN>UDP    CANID 0x%08X   [%d]";
char *TCP_FORMAT_STRG      = "->TCP>CAN    CANID 0x%08X   [%d]";
char *TCP_FORMATS_STRG     = "->TCP>CAN*   CANID 0x%08X   [%d]";
char *CAN_TCP_FORMAT_STRG  = "->CAN>TCP    CANID 0x%08X   [%d]";
char *NET_UDP_FORMAT_STRG  = "      UDP->  CANID 0x%08X   [%d]";

unsigned char M_GLEISBOX_MAGIC_START_SEQUENCE[] = { 0x00, 0x36, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00 };
unsigned char M_PING_RESPONSE[] = { 0x00, 0x30, 0x00, 0x00, 0x00 };

char config_dir[MAXLINE] = "cs2";
char config_file[MAXLINE];
char **page_name;
int verbose = 0;
int ms1_workaround = 0;

void print_usage(char *prg) {
    fprintf(stderr, "\nUsage: %s -c <config_dir> -u <udp_port> -t <tcp_port> -d <udp_dest_port> -i <can interface>\n", prg);
    fprintf(stderr, "   Version 1.01 bh\n\n");
    fprintf(stderr, "         -c <config_dir>     set the config directory - default cs2\n");
    fprintf(stderr, "         -u <port>           listening UDP port for the server - default 15731\n");
    fprintf(stderr, "         -t <port>           listening TCP port for the server - default 15731\n");
    fprintf(stderr, "         -d <port>           destination UDP port for the server - default 15730\n");
    fprintf(stderr, "         -b <bcast_addr/int> broadcast address or interface - default 255.255.255.255/br-lan\n");
//    fprintf(stderr, "         -i <can int>        CAN interface - default can0\n");
    fprintf(stderr, "         -i <arduino port>   incoming from arduino interface - default 35730\n");
    fprintf(stderr, "         -a <arduino port>   calling port to arduino interface - default 35731\n");

    fprintf(stderr, "         -m                  doing MS1 workaround - default: don't do it\n\n");
    fprintf(stderr, "         -f                  running in foreground\n\n");
    fprintf(stderr, "         -v                  verbose output (in foreground)\n\n");
}

/*int send_magic_start_60113_frame(int can_socket) {

//dit omzetten naar udp tbv arduino
//
    if (frame_to_can(can_socket, M_GLEISBOX_MAGIC_START_SEQUENCE) < 0) {
	fprintf(stderr, "can't send CAN magic 60113 start sequence\n");
	return -1;
    } else {
	if (verbose) {
	    printf("                CAN magic 60113 start written\n");
	    print_can_frame(CAN_FORMAT_STRG, M_GLEISBOX_MAGIC_START_SEQUENCE, verbose);
	}
    }
    return 0;
}
*/
		/* send UDP frame to arduino */
/*
    if (frame_to_net(sd, (struct sockaddr *)&asaddr, (struct can_frame *)&frame) <0 {
	fprintf(stderr, "can't send CAN magic 60113 start sequence\n");
	return -1;
    } else {
	if (verbose) {
	    printf("                CAN magic 60113 start written\n");
//	    print_can_frame(CAN_FORMAT_STRG, M_GLEISBOX_MAGIC_START_SEQUENCE, verbose);
// deze hoorde hier niet		print_can_frame(UDP_FORMAT_STRG, netframe, verbose &!background);
 	  print_can_frame(NET_UDP_FORMAT_STRG, netframe, verbose & !background);
	}
    }
    return 0;
}
*/

int check_data(int tcp_socket, unsigned char *netframe) {
    uint32_t canid;
    char config_name[9];
    char gbs_name[MAXLINE];
    gbs_name[0] = '\0';
    int ret=0;

    memcpy(&canid, netframe, 4);
    canid = ntohl(canid);
    if (verbose) {
      printf("can2lan-093 CAN  received, header: 0x%08x\n", canid);
    }

    switch (canid & 0xFFFF0000UL) {
    case (0x00400000UL):	/* config data */
	/* marke CAN frame not to send */
	ret = 1;
	strncpy(config_name, (char *)&netframe[5], 8);
	config_name[8] = '\0';
	printf("can2lan-098 function: %s ID 0x%08x %s\n", __func__, canid, (char *)&netframe[5]);
	netframe[1] |= 1;
	net_to_net(tcp_socket, NULL, netframe, 13);
	if (strcmp("loks", config_name) == 0) {
	    send_tcp_config_data("lokomotive.cs2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	} else if (strcmp("mags", config_name) == 0) {
	    send_tcp_config_data("magnetartikel.cs2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	} else if (strncmp("gbs-", config_name, 4) == 0) {
	    int page_number;
	    page_number = atoi(&config_name[5]);
	    strcat(gbs_name, "gleisbilder/");
	    if (page_name) {
		strcat(gbs_name, page_name[page_number]);
		strcat(gbs_name, ".cs2");
		send_tcp_config_data(gbs_name, config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    }
	    break;
	} else if (strcmp("gbs", config_name) == 0) {
	    send_tcp_config_data("gleisbild.cs2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	} else if (strcmp("fs", config_name) == 0) {
	    send_tcp_config_data("fahrstrassen.cs2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	}
	/* TODO : these files depends on different internal states */
	else if (strcmp("lokstat", config_name) == 0) {
	    fprintf(stderr, "can2lan-126 %s: lokstat (lokomotive.sr2) not implemented yet\n", __func__);
	    send_tcp_config_data("lokomotive.sr2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	} else if (strcmp("magstat", config_name) == 0) {
	    fprintf(stderr, "can2lan-130 %s: magstat (magnetartikel.sr2) not implemented yet\n\n", __func__);
	    send_tcp_config_data("magnetartikel.sr2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	} else if (strcmp("gbsstat", config_name) == 0) {
	    fprintf(stderr, "can2lan-134 %s: gbsstat (gbsstat.sr2) not implemented yet\n\n", __func__);
	    send_tcp_config_data("gbsstat.sr2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	} else if (strcmp("fsstat", config_name) == 0) {
	    fprintf(stderr, "can2lan-138 %s: fsstat (fahrstrassen.sr2) not implemented yet\n\n", __func__);
	    send_tcp_config_data("fahrstrassen.sr2", config_dir, canid, tcp_socket, CRC | COMPRESSED);
	    break;
	}
        break;
    /* fake cyclic MS1 slave monitoring response */
    case (0x0C000000UL):
	/* mark CAN frame to send */
	ret = 0;
	if (ms1_workaround)
	    netframe[5] = 0x03;
        break;
    }
    return ret;
}

int main(int argc, char **argv) {
    pid_t pid;
    /* struct cs2_config cs2_config; */
    extern int optind, opterr, optopt;
    int n, i, max_fds, opt, max_tcp_i, nready, conn_fd, tcp_client[MAX_TCP_CONN];;
//    struct can_frame frame;
    char timestamp[16];

//    int sa, sc, sb, st, tcp_socket;	/* UDP incoming socket , CAN socket, UDP broadcast socket, TCP socket */
    int sa, sca, sba, sb, st, tcp_socket;	/* UDP incoming socket , incoming arduino socket, outgoing broadcast Arduino socket, UDP broadcast socket, TCP socket */
    struct sockaddr_in saddr, baddr, araddr, asaddr, tcp_addr;
    /* vars for determing broadcast address */
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *bsa;

//    struct sockaddr_can caddr;
//    struct ifreq ifr;
//    socklen_t caddrlen = sizeof(caddr);
    socklen_t tcp_client_length = sizeof(tcp_addr);

    fd_set all_fds, read_fds;

    uint32_t canid;
    int s, ret;

// can port omzetten naar een udp port naar de arduino
    int arduino_udp_port = 35730;
    int dest_arduino_udp_port = 35731;
    int local_udp_port = 15731;
    int local_tcp_port = 15731;
    int destination_port = 15730;
    int background = 0;
    const int on = 1;
    char buffer[64];
    page_name = calloc(64, sizeof(char *));

    /* TODO : where to use */
    char *udp_dst_address = (char*)malloc(16);
    strcpy(udp_dst_address, "255.255.255.255");
    char *bcast_interface = (char*)malloc(16);
    strcpy(bcast_interface, "wlan0");

    config_file[0] = '\0';

    while ((opt = getopt(argc, argv, "c:u:t:d:b:a:i:mvhf?")) != -1) {
	switch (opt) {
	case 'c':
	    if (strlen(optarg) < MAXLINE) {
		strcpy(config_dir, optarg);
	    } else {
		fprintf(stderr, "can2lan-213 config file dir to long\n");
		exit(1);
	    }
	    break;
	case 'u':
	    local_udp_port = strtoul(optarg, (char **)NULL, 10);
	    break;
	case 't':
	    local_tcp_port = strtoul(optarg, (char **)NULL, 10);
	    break;
	case 'd':
	    destination_port = strtoul(optarg, (char **)NULL, 10);
	    break;
	case 'b':
	    if (strlen(optarg) <= 15) {
		/* IP address begins with a number */
		if ((optarg[0] >= '0') || (optarg[0] <= '9'))
		    strcpy(udp_dst_address, optarg);
		else
		    bzero(bcast_interface, 16);
		    strcpy(bcast_interface, optarg);
	    } else {
		fprintf(stderr, "can2lan-235 UDP broadcast address or interface error: %s\n", optarg);
		exit(1);
	    }
	    break;
// deze case vervangen door een poort naar de arduino
	case 'i':
//	    strcpy(ifr.ifr_name, optarg);
		dest_arduino_udp_port = strtoul(optarg, (char **)NULL, 10);
	    break;
	case 'a':
		arduino_udp_port = strtoul(optarg, (char **)NULL, 10);
	    break;
	case 'm':
	    ms1_workaround = 1;
	    break;
	case 'v':
	    verbose = 1;
	    break;
	case 'f':
	    background = 0;
	    break;
	case 'h':
	case '?':
	    print_usage(basename(argv[0]));
	    exit(0);
	default:
	    fprintf(stderr, "can2lan-261 Unknown option %c\n", opt);
	    print_usage(basename(argv[0]));
	    exit(1);
	}
    }

    /* read track file */
    if (config_dir[0] == 0) {
	strcat(config_dir, ".");
    }
    if (config_dir[strlen(config_dir)] != '/') {
	strcat(config_dir, "/");
    }
    strcat(config_file, config_dir);
    strcat(config_file, "gleisbild.cs2");

    page_name = read_track_file(config_file, page_name);

    /* get the broadcast address */
    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
	if (ifa->ifa_addr) {
	    if (ifa->ifa_addr->sa_family==AF_INET) {
		bsa = (struct sockaddr_in *) ifa->ifa_broadaddr;
		if (strncmp(ifa->ifa_name, bcast_interface, strlen(bcast_interface)) == 0)
		    udp_dst_address = inet_ntoa(bsa->sin_addr);
	    }
	}
    }

    /* prepare udp sending socket struct */
    bzero(&baddr, sizeof(baddr));
    baddr.sin_family = AF_INET;
    baddr.sin_port = htons(destination_port);
    s = inet_pton(AF_INET, udp_dst_address, &baddr.sin_addr);
    if (s <= 0) {
	if (s == 0) {
	    fprintf(stderr, "can2lan-298 UDP IP address invalid\n");
	} else {
	    fprintf(stderr, "can2lan-300 invalid address family\n");
	}
	exit(1);
    }

    if (verbose & !background)
	printf("can2lan-306 using broadcast address %s\n", udp_dst_address);

// ingevoegd
    /* prepare udp sending to arduino socket struct */
    bzero(&asaddr, sizeof(asaddr));
    asaddr.sin_family = AF_INET;
    asaddr.sin_port = htons(dest_arduino_udp_port);

    s = inet_pton(AF_INET, udp_dst_address, &asaddr.sin_addr);
    if (s <= 0) {
	if (s == 0) {
	    fprintf(stderr, "can2lan-317 UDP IP address invalid\n");
	} else {
	    fprintf(stderr, "can2lan-319 invalid address family\n");
	}
	exit(1);
    }

    if (verbose & !background)
	printf("can2lan-325 using Arduino address %s\n", udp_dst_address);

//
    /* prepare UDP sending socket */
    if ((sb = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	fprintf(stderr, "can2lan-330 error creating UDP sending socket: %s\n", strerror(errno));
	exit(1);
    }
    if (setsockopt(sb, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) < 0) {
	fprintf(stderr, "can2lan-334 error setup UDP broadcast option: %s\n", strerror(errno));
	exit(1);
    }

    /* prepare Arduino UDP sending socket */
    if ((sba = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	fprintf(stderr, "can2lan-340 error creating Arduino UDP sending socket: %s\n", strerror(errno));
	exit(1);
    }
    if (setsockopt(sba, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) < 0) {
	fprintf(stderr, "can2lan-344 error setup Arduino UDP broadcast option: %s\n", strerror(errno));
	exit(1);
    }


    /* prepare reading UDP socket */
    bzero(&saddr, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(local_udp_port);
    if ((sa = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
	fprintf(stderr, "can2lan-355 creating UDP reading socket error: %s\n", strerror(errno));
	exit(1);
    }
    if (bind(sa, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
	fprintf(stderr, "can2lan-359 binding UDP reading socket error: %s address: 0x%08X \n", strerror(errno), INADDR_ANY);
	exit(1);
    }

//ingevoegd
    /* prepare reading Arduino UDP socket */
    bzero(&araddr, sizeof(araddr));
    araddr.sin_family = AF_INET;
    araddr.sin_addr.s_addr = htonl(INADDR_ANY);
    araddr.sin_port = htons(arduino_udp_port);
    if ((sca = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
	fprintf(stderr, "can2lan-370 creating Arduino UDP reading socket error: %s\n", strerror(errno));
	exit(1);
    }
    if (bind(sca, (struct sockaddr *)&araddr, sizeof(araddr)) < 0) {
	fprintf(stderr, "can2lan-374 binding UDP for Arduino reading socket error: %s\n", strerror(errno));
	exit(1);
    }
//
    /* prepare TCP socket */
    if ((st = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
	fprintf(stderr, "can2lan-380 creating TCP socket error: %s\n", strerror(errno));
	exit(1);
    }
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcp_addr.sin_port = htons(local_tcp_port);
    if (bind(st, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0) {
	fprintf(stderr, "can2lan-387 binding TCP socket error: %s\n", strerror(errno));
	exit(1);
    }
    if (listen(st, MAXPENDING) < 0) {
	fprintf(stderr, "can2lan-391 starting TCP listener error: %s\n", strerror(errno));
	exit(1);
    }
    /* prepare TCP clients array */
    max_tcp_i = -1;		/* index into tcp_client[] array */
    for (i = 0; i < MAX_TCP_CONN; i++)
	tcp_client[i] = -1;	/* -1 indicates available entry */

// dit deel ook vrvangen door een udp sending en receive port tbv de arduino
/* start Maerklin 60113 box */
/* send UDP frame to arduino */

    if (net_to_net(sba, (struct sockaddr *)&asaddr, M_GLEISBOX_MAGIC_START_SEQUENCE, 13) <0 ) {
	fprintf(stderr, "can2lan-404 can't send CAN magic 60113 start sequence\n");
    }
    else {
      if (verbose) {
	printf("can2lan-408 CAN magic 60113 start written\n");
        print_can_frame(CAN_FORMAT_STRG, M_GLEISBOX_MAGIC_START_SEQUENCE, verbose);
      }
   }

    /* daemonize the process if requested */
    if (background) {
	/* fork off the parent process */
	pid = fork();
	if (pid < 0) {
	    exit(EXIT_FAILURE);
	}
	/* if we got a good PID, then we can exit the parent process */
	if (pid > 0) {
	    printf("can2lan-422 Going into background ...\n");
	    exit(EXIT_SUCCESS);
	}
    }

    FD_ZERO(&all_fds);
    FD_SET(sca, &all_fds);
    FD_SET(sa, &all_fds);
//FD_SET(sba, &all_fds);
    FD_SET(st, &all_fds);
    max_fds = MAX(MAX(sca, sa), st);

    while (1) {
	read_fds = all_fds;
	if ((nready = select(max_fds + 1, &read_fds, NULL, NULL, NULL)) < 0) {
	    fprintf(stderr, "can2lan-437 select error: %s\n", strerror(errno));
	}

	/* received a arduino CAN frame */
	if (FD_ISSET(sca, &read_fds)) {
	  if (read(sca, netframe, MAXDG) != 13) {
	    fprintf(stderr, "can2lan-443 reading CAN frame from Arduino, error: %s\n", strerror(errno));
	  }
	  /* send UDP frame */
	  net_to_net(sb, (struct sockaddr *)&baddr, netframe, 13);
	  print_can_frame(UDP_FORMAT_STRG, netframe, verbose &!background);
	  /* send CAN frame to all connected TCP clients */
	  /* TODO: need all clients the packets ? */
	  for (i = 0; i <= max_tcp_i; i++) {	/* check all clients for data */
	    if ((tcp_socket = tcp_client[i]) < 0)
		continue;
	    net_to_net(tcp_socket, (struct sockaddr *)&tcp_addr, netframe, 13);
	    print_can_frame(CAN_TCP_FORMAT_STRG, netframe, verbose & !background);
	  }
	}
	/* received a UDP packet */
	if (FD_ISSET(sa, &read_fds)) {
	    if (read(sa, netframe, MAXDG) == 13) {
		/* send packet on CAN */
		/* send UDP frame to arduino */
		net_to_net(sba, (struct sockaddr *)&asaddr, netframe, 13);
                printf("can2lan-463 printing CAN frame with UDP_FORMAT_STRG: \n");
 		print_can_frame(NET_UDP_FORMAT_STRG, netframe, verbose &!background);
//                printf("can2lan-465 printing CAN frame with UDP_FORMAT_STRG: \n");
//		print_can_frame(NET_UDP_FORMAT_STRG, netframe, verbose & !background);

		memcpy(&canid, netframe, 4);
		canid = ntohl(canid);

		/* answer to encapsulated CAN ping from LAN to LAN */
		if (((canid & 0xFFFF0000UL) == 0x00310000UL)
		    && (netframe[11] = 0xEE) && (netframe[12] = 0xEE)) {
		    if (verbose & !background)
			printf("can2lan-475 received CAN ping\n");
		    memcpy(netframe, M_PING_RESPONSE, 5);
		    if (net_to_net(sb, (struct sockaddr *)&baddr, netframe, 13)) {
			fprintf(stderr, "can2lan-478 sending UDP data (CAN Ping) error:%s \n", strerror(errno));
		    } else {
			print_can_frame(NET_UDP_FORMAT_STRG, netframe, verbose & !background);
			if (verbose & !background)
			    printf("can2lan-482 replied CAN ping\n");
		    }
		}
	    }
	}
	/* received a TCP packet */
	if (FD_ISSET(st, &read_fds)) {
	    conn_fd = accept(st, (struct sockaddr *)&tcp_addr, &tcp_client_length);
	    if (verbose && !background) {
		printf("can2lan-491 new client: %s, port %d conn fd: %d max fds: %d\n", inet_ntop(AF_INET, &(tcp_addr.sin_addr),
			buffer, sizeof(buffer)), ntohs(tcp_addr.sin_port), conn_fd, max_fds);
	    }
	    for (i = 0; i < MAX_TCP_CONN; i++) {
		if (tcp_client[i] < 0) {
		    tcp_client[i] = conn_fd;	/* save new TCP client descriptor */
		    break;
		}
	    }
	    if (i == MAX_TCP_CONN)
		fprintf(stderr, "can2lan-501 too many TCP clients\n");

	    FD_SET(conn_fd, &all_fds);	/* add new descriptor to set */
	    max_fds = MAX(conn_fd, max_fds);	/* for select */
	    max_tcp_i = MAX(i, max_tcp_i);	/* max index in tcp_client[] array */
	    if (--nready <= 0)
		continue;	/* no more readable descriptors */
	}
	/* check for already connected TCP clients */
	for (i = 0; i <= max_tcp_i; i++) {	/* check all clients for data */
	    if ((tcp_socket = tcp_client[i]) < 0)
		continue;
	    /* printf("%s tcp packet received from client #%d  max_tcp_i:%d todo:%d\n", time_stamp(timestamp), i, max_tcp_i,nready); */
	    if (FD_ISSET(tcp_socket, &read_fds)) {
		if (verbose && !background) {
		    time_stamp(timestamp);
		    printf("%s can2lan-517: TCP packet from: %s\n", timestamp, inet_ntop(AF_INET, &tcp_addr.sin_addr, buffer, sizeof(buffer)));
		}
		if ((n = read(tcp_socket, netframe, MAXDG)) == 0) {
		    /* connection closed by client */
		    if (verbose && !background) {
			time_stamp(timestamp);
			printf("%s can2lan-523: client %s closed connection\n", timestamp,
			    inet_ntop(AF_INET, &tcp_addr.sin_addr, buffer, sizeof(buffer)));
		    }
		    close(tcp_socket);
		    FD_CLR(tcp_socket, &all_fds);
		    tcp_client[i] = -1;
		} else {
		    /* check the whole TCP packet, if there are more than one CAN frame included */
		    /* TCP packets with size modulo 13 !=0 are ignored though */
		    if (n % 13) {
			time_stamp(timestamp);
			fprintf(stderr, "%s can2lan-534: received packet %% 13 : length %d\n", timestamp, n);
		    } else {
			for (i = 0; i < n; i += 13) {
			    /* check if we need to forward the message to CAN */
			    if (!check_data(tcp_socket, &netframe[i])) {
// dit omzetten naar arduino poort
/*
				if ((ret = frame_to_can(sc, &netframe[i])) == 0) {
				    if (i > 0)
					print_can_frame(TCP_FORMATS_STRG, &netframe[i], verbose & !background);
				    else
					print_can_frame(TCP_FORMAT_STRG, &netframe[i], verbose & !background);
				}
*/
		                ret = net_to_net(sba, (struct sockaddr *)&asaddr, netframe, 13);
                                if  ((ret == 0) && (i > 0))
					print_can_frame(TCP_FORMATS_STRG, &netframe[i], verbose & !background);
				    else
					print_can_frame(TCP_FORMAT_STRG, &netframe[i], verbose & !background);

			    }
			}
		    }
		}
		if (--nready <= 0)
		    break;	/* no more readable descriptors */
	    }
	}
    }
    close(sca);
    close(sa);
    close(sb);
    close(sba);
    close(st);
    return 0;
}
