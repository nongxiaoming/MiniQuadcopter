## Flag that can be added to config.mk
# CFLAGS += -DUSE_UART_CRTP        # Set CRTP link to UART
# CFLAGS += -DUSE_ESKYLINK         # Set CRTP link to E-SKY receiver
CFLAGS += -DENABLE_UART          # To enable the uart
CFLAGS += -DDEBUG_PRINT_ON_UART  # Redirect the console output to the UART
# CFLAGS += -DENABLE_FAST_CHARGE   # Will enable ~800mA USB current for wall adapters. Should only be used with batteries 
# that can handle ~740mA charge current or it will degrade the battery.