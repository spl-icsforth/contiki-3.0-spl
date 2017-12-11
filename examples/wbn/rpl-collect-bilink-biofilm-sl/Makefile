CONTIKI = ../../..
#######################################
##uncomment to use with xm1000#####
#CONTIKI_TARGET_SOURCEFILES += uart0.c
####################################

########################################################################
## uncomment to use null rdc (with FORTH addons)
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
########################################################################

CONTIKI_WITH_UIP6=1
UIP_CONF_IPV6=1

CFLAGS+= -DUIP_CONF_IPV6_RPL

ifdef PERIOD
CFLAGS=-DPERIOD=$(PERIOD)
endif

########################################################################
##uncomment to:
#a. record tx/rx MAC counters
CFLAGS+= -DNM
########################################################################
#b. AND record the ids of the relays (2 subsequent lines)
CFLAGS+= -DTRACE_ROUTE
PROJECT_SOURCEFILES += nm-common.c
#c. AND record the PHY metrics (RSSI, LQI, NF) per relay)
CFLAGS+= -DTRACE_LQ
########################################################################
CONTIKI_SOURCEFILES += nullrdc_dl.c
#csma-uveg-v2.c 
#c. OR, uncomment both a and b in order to use the default csma.

##############################################################################

##unocomment to activate downlink communication...
CFLAGS+= -DWITH_DL
TARGET_LIBFILES += -lm
#########################################################
###############################################################################
#Set the ID of each node (currently: Sink->16, Sources:1-10, Relays: 11-15)
#comment when testing at cooja:
#CFLAGS+= -DNODEID=11

##for sink:
#uncomment to write directly to serial port (no printf) for the data collection (Note: comment when testing at cooja)

#CFLAGS+= -DSERIAL_MODE
#########################################################
##for connectivity with biofilm -
#sensor1  - sensor2 
#CFLAGS+= -DBFMODE1
#CFLAGS+= -DBFMODE2
#uncomment for connection with arduino pro - setting baudrate to 19200
#CFLAGS+= -DBIOFILM
###############################################################################

include $(CONTIKI)/Makefile.include
