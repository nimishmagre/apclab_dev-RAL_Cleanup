execute_process(COMMAND "/home/nimish/apclab_dev-RAL_Cleanup/build/hardware_drivers/arg2f_driver/robotiq_modbus_rtu/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nimish/apclab_dev-RAL_Cleanup/build/hardware_drivers/arg2f_driver/robotiq_modbus_rtu/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
