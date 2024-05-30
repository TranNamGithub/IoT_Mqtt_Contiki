#all: main_core
all: co2andradar falling breathing cc2538dk_shield
PROJECT_SOURCEFILES += mqtt_sn.c

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

# Adicionada estas duas linhas de flags para reduzir tamanho do firmware que não cabe no espaço de rom do msp430 que é utilizado na simulação
#CFLAGS += -ffunction-sections
#LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__
#Nam add lib for cc2538sheild
CFLAGS += -DCC2538DK_HAS_SENSOR=1
CONTIKI_TARGET_SOURCEFILES += tsl256x.c bmpx8x.c si7021.c
MODULES += platform/cc2538dk/dev
# end
CONTIKI=../
include $(CONTIKI)/Makefile.include
