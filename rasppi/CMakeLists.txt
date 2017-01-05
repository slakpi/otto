project("otto")

add_executable(otto ../Autopilot.cpp
                    ../AveragingBuffer.cpp
                    ../DataSource.cpp
                    ../FlightDirector.cpp
                    ../GISDatabase.cpp
                    ../Utilities.cpp
                    ./Arduino.cpp
                    ./HD44780.cpp
                    ./LIS3MDL.cpp
                    ./LSM6DS33.cpp
                    ./Madgwick_AHRS.c
                    ./NMEA.cpp
                    ./rasppi.cpp
                    ./RpiAutopilot.cpp
                    ./RpiDataSource.cpp)
target_compile_features(otto PRIVATE cxx_nullptr)
target_include_directories(otto PRIVATE ./ ../)
target_link_libraries(otto sqlite3 spatialite wiringPi pthread)

add_executable(rdbtool ../nav/recoverydb.cpp)
target_compile_features(rdbtool PRIVATE cxx_nullptr)
target_link_libraries(rdbtool sqlite3 spatialite)

add_custom_command(OUTPUT recovery.db
                    DEPENDS rdbtool
                    COMMAND $<TARGET_FILE:rdbtool> recovery.db $<TARGET_FILE_DIR:rdbtool>/../../nav/recovery.csv
					COMMENT "Building recovery database"
					VERBATIM)
add_custom_target(recovery_db DEPENDS recovery.db)

add_custom_target(tests DEPENDS test_arduino test_gps test_imu test_mag)

add_executable(test_arduino EXCLUDE_FROM_ALL ../AveragingBuffer.cpp
                    ../DataSource.cpp
                    ./Arduino.cpp
                    ./HD44780.cpp
                    ./LIS3MDL.cpp
                    ./LSM6DS33.cpp
                    ./Madgwick_AHRS.c
                    ./NMEA.cpp
                    ./RpiDataSource.cpp
					./tests/test_arduino.cpp)
target_compile_features(test_arduino PRIVATE cxx_nullptr)
target_include_directories(test_arduino PRIVATE ./ ../)
target_link_libraries(test_arduino wiringPi pthread)

add_executable(test_gps EXCLUDE_FROM_ALL ./HD44780.cpp
                    ./NMEA.cpp
					./tests/test_gps.cpp)
target_compile_features(test_gps PRIVATE cxx_nullptr)
target_include_directories(test_gps PRIVATE ./ ../)
target_link_libraries(test_gps wiringPi)

add_executable(test_imu EXCLUDE_FROM_ALL ../AveragingBuffer.cpp
                    ./HD44780.cpp
                    ./LIS3MDL.cpp
                    ./LSM6DS33.cpp
                    ./Madgwick_AHRS.c
					./tests/test_imu.cpp)
target_compile_features(test_imu PRIVATE cxx_nullptr)
target_include_directories(test_imu PRIVATE ./ ../)
target_link_libraries(test_imu wiringPi)

add_executable(test_mag EXCLUDE_FROM_ALL ./LIS3MDL.cpp
					./tests/test_mag.cpp)
target_compile_features(test_mag PRIVATE cxx_nullptr)
target_include_directories(test_mag PRIVATE ./ ../)
target_link_libraries(test_mag wiringPi)
