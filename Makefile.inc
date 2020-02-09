APPL_COBJS += 

APPL_CXXOBJS += second.o

SRCLANG := c++

CC = g++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-test/Makefile
include $(EV3RT_SDK_LIB_DIR)/lib2/Makefile

endif
