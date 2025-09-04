# Findings

Some initial findings that were triggered when fuzzing the SDK for a short while.

## 1. Multiple memory Leaks

```
INFO: Running with entropic power schedule (0xFF, 100).
INFO: Seed: 2732911167
INFO: Loaded 2 modules   (3982 inline 8-bit counters): 3655 [0x7e0b2a7ef058, 0x7e0b2a7efe9f), 327 [0x6482bb2b05d8, 0x6482bb2b071f),
INFO: Loaded 2 PC tables (3982 PCs): 3655 [0x7e0b2a7efea0,0x7e0b2a7fe310), 327 [0x6482bb2b0720,0x6482bb2b1b90),
INFO: -max_len is not provided; libFuzzer will not generate inputs larger than 4096 bytes
INFO: A corpus is not provided, starting from an empty corpus
#2      INITED cov: 85 ft: 86 corp: 1/1b exec/s: 0 rss: 32Mb

=================================================================
==685022==ERROR: LeakSanitizer: detected memory leaks

Direct leak of 24 byte(s) in 1 object(s) allocated from:
    #0 0x6482bb2633e1 in operator new(unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x1423e1) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #1 0x7e0b2a7bd358 in rp::standalone::rplidar::RPlidarDriverSerial::RPlidarDriverSerial() /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:2181:16
    #2 0x7e0b2a79cebd in rp::standalone::rplidar::RPlidarDriver::CreateDriver(unsigned int) /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:89:20
    #3 0x6482bb265959 in LLVMFuzzerTestOneInput /home/user/git/work/rpilidar_ros2_clean/fuzz/harness.cc:38:14
    #4 0x6482bb170cb4 in fuzzer::Fuzzer::ExecuteCallback(unsigned char const*, unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4fcb4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #5 0x6482bb1703a9 in fuzzer::Fuzzer::RunOne(unsigned char const*, unsigned long, bool, fuzzer::InputInfo*, bool, bool*) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4f3a9) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #6 0x6482bb172036 in fuzzer::Fuzzer::ReadAndExecuteSeedCorpora(std::vector<fuzzer::SizedFile, std::allocator<fuzzer::SizedFile>>&) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x51036) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #7 0x6482bb1724d7 in fuzzer::Fuzzer::Loop(std::vector<fuzzer::SizedFile, std::allocator<fuzzer::SizedFile>>&) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x514d7) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #8 0x6482bb15f9cf in fuzzer::FuzzerDriver(int*, char***, int (*)(unsigned char const*, unsigned long)) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x3e9cf) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #9 0x6482bb18a056 in main (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x69056) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #10 0x7e0b2a42a1c9 in __libc_start_call_main csu/../sysdeps/nptl/libc_start_call_main.h:58:16
    #11 0x7e0b2a42a28a in __libc_start_main csu/../csu/libc-start.c:360:3
    #12 0x6482bb1549b4 in _start (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x339b4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)

Direct leak of 24 byte(s) in 1 object(s) allocated from:
    #0 0x6482bb2633e1 in operator new(unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x1423e1) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #1 0x7e0b2a7bd358 in rp::standalone::rplidar::RPlidarDriverSerial::RPlidarDriverSerial() /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:2181:16
    #2 0x7e0b2a79cebd in rp::standalone::rplidar::RPlidarDriver::CreateDriver(unsigned int) /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:89:20
    #3 0x6482bb265959 in LLVMFuzzerTestOneInput /home/user/git/work/rpilidar_ros2_clean/fuzz/harness.cc:38:14
    #4 0x6482bb170cb4 in fuzzer::Fuzzer::ExecuteCallback(unsigned char const*, unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4fcb4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #5 0x6482bb1703a9 in fuzzer::Fuzzer::RunOne(unsigned char const*, unsigned long, bool, fuzzer::InputInfo*, bool, bool*) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4f3a9) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #6 0x6482bb171b95 in fuzzer::Fuzzer::MutateAndTestOne() (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x50b95) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #7 0x6482bb1726f5 in fuzzer::Fuzzer::Loop(std::vector<fuzzer::SizedFile, std::allocator<fuzzer::SizedFile>>&) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x516f5) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #8 0x6482bb15f9cf in fuzzer::FuzzerDriver(int*, char***, int (*)(unsigned char const*, unsigned long)) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x3e9cf) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #9 0x6482bb18a056 in main (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x69056) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #10 0x7e0b2a42a1c9 in __libc_start_call_main csu/../sysdeps/nptl/libc_start_call_main.h:58:16
    #11 0x7e0b2a42a28a in __libc_start_main csu/../csu/libc-start.c:360:3
    #12 0x6482bb1549b4 in _start (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x339b4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)

Direct leak of 24 byte(s) in 1 object(s) allocated from:
    #0 0x6482bb2633e1 in operator new(unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x1423e1) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #1 0x7e0b2a7bd358 in rp::standalone::rplidar::RPlidarDriverSerial::RPlidarDriverSerial() /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:2181:16
    #2 0x7e0b2a79cebd in rp::standalone::rplidar::RPlidarDriver::CreateDriver(unsigned int) /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:89:20
    #3 0x6482bb265959 in LLVMFuzzerTestOneInput /home/user/git/work/rpilidar_ros2_clean/fuzz/harness.cc:38:14
    #4 0x6482bb170cb4 in fuzzer::Fuzzer::ExecuteCallback(unsigned char const*, unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4fcb4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #5 0x6482bb171ee1 in fuzzer::Fuzzer::ReadAndExecuteSeedCorpora(std::vector<fuzzer::SizedFile, std::allocator<fuzzer::SizedFile>>&) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x50ee1) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #6 0x6482bb1724d7 in fuzzer::Fuzzer::Loop(std::vector<fuzzer::SizedFile, std::allocator<fuzzer::SizedFile>>&) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x514d7) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #7 0x6482bb15f9cf in fuzzer::FuzzerDriver(int*, char***, int (*)(unsigned char const*, unsigned long)) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x3e9cf) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #8 0x6482bb18a056 in main (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x69056) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #9 0x7e0b2a42a1c9 in __libc_start_call_main csu/../sysdeps/nptl/libc_start_call_main.h:58:16
    #10 0x7e0b2a42a28a in __libc_start_main csu/../csu/libc-start.c:360:3
    #11 0x6482bb1549b4 in _start (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x339b4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)

SUMMARY: AddressSanitizer: 72 byte(s) leaked in 3 allocation(s).
INFO: to ignore leaks on libFuzzer side use -detect_leaks=0.
```

### Possible fix

It seems to help to extend the destructor of the objects with a proper delete operation, similar to:

```cc
RPlidarDriverSerial::~RPlidarDriverSerial() {
  // force disconnection
  disconnect();

  _chanDev->close();
  _chanDev->ReleaseRxTx();

  // NOTE: prevent memory leak
  delete _chanDev;
  _chanDev = nullptr;
}
```

_Note_: This also needs to be done for the `RPlidarDriverTCP::~RPlidarDriverTCP()`

## 2. NULL-PTR access

```
#1887   NEW    cov: 265 ft: 288 corp: 14/107b lim: 21 exec/s: 0 rss: 105Mb L: 18/21 MS: 2 ChangeBit-CrossOver-
/home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver_TCP.h:48:16: runtime error: member call on null pointer of type 'rp::net::StreamSocket'
SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver_TCP.h:48:16
AddressSanitizer:DEADLYSIGNAL
=================================================================
==688768==ERROR: AddressSanitizer: SEGV on unknown address 0x000000000000 (pc 0x7532a0b5440f bp 0x7fff999e3af0 sp 0x7fff999e3a60 T0)
==688768==The signal is caused by a READ memory access.
==688768==Hint: address points to the zero page.
    #0 0x7532a0b5440f in rp::standalone::rplidar::TCPChannelDevice::bind(char const*, unsigned int) /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver_TCP.h:48:16
    #1 0x7532a0b5251b in rp::standalone::rplidar::RPlidarDriverTCP::connect(char const*, unsigned int, unsigned int) /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver.cpp:2251:23
    #2 0x595daaac4017 in LLVMFuzzerTestOneInput /home/user/git/work/rpilidar_ros2_clean/fuzz/harness.cc:60:12
    #3 0x595daa9cecb4 in fuzzer::Fuzzer::ExecuteCallback(unsigned char const*, unsigned long) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4fcb4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #4 0x595daa9ce3a9 in fuzzer::Fuzzer::RunOne(unsigned char const*, unsigned long, bool, fuzzer::InputInfo*, bool, bool*) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x4f3a9) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #5 0x595daa9cfb95 in fuzzer::Fuzzer::MutateAndTestOne() (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x50b95) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #6 0x595daa9d06f5 in fuzzer::Fuzzer::Loop(std::vector<fuzzer::SizedFile, std::allocator<fuzzer::SizedFile>>&) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x516f5) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #7 0x595daa9bd9cf in fuzzer::FuzzerDriver(int*, char***, int (*)(unsigned char const*, unsigned long)) (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x3e9cf) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #8 0x595daa9e8056 in main (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x69056) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)
    #9 0x7532a042a1c9 in __libc_start_call_main csu/../sysdeps/nptl/libc_start_call_main.h:58:16
    #10 0x7532a042a28a in __libc_start_main csu/../csu/libc-start.c:360:3
    #11 0x595daa9b29b4 in _start (/home/user/git/work/rpilidar_ros2_clean/fuzz/build/harness+0x339b4) (BuildId: 7c06b0c5fb4572cd2125cd2971d87e1a2e416f2f)

AddressSanitizer can not provide additional info.
SUMMARY: AddressSanitizer: SEGV /home/user/git/work/rpilidar_ros2_clean/sdk/src/rplidar_driver_TCP.h:48:16 in rp::standalone::rplidar::TCPChannelDevice::bind(char const*, unsigned int)
==688768==ABORTING
MS: 5 ChangeBinInt-ChangeBinInt-CopyPart-EraseBytes-ChangeByte-; base unit: 194f4b33f5c6e20eeab91e4f162f8f5bd27143c9
0xfd,0x0,0x0,0x2,0x2,0xe5,
\375\000\000\002\002\345
artifact_prefix='./'; Test unit written to ./crash-ce81a181206832f1dc742353c3e3c8ed137bcf1e
Base64: /QAAAgLl
```

The root-cause here seems to be that if we call `drv.connect()` for a `DRIVER_TYPE_TCP` like so:

```cc
drv->connect(ip.c_str(), port);
```

on the condition that we did _not_ check _\_bindet_socket_ for NULL we can crash the process.

### Possible fix:

```cc
  TCPChannelDevice() : _binded_socket(rp::net::StreamSocket::CreateSocket()) {
    // NOTE: Include a sanity NULLPTR check
    if (_binded_socket == nullptr) {
      std::cerr << "Failed to create socket" << std::endl;
    }
  }
  bool bind(const char *ipStr, uint32_t port) {
    if (_binded_socket == nullptr) {
      std::cerr << "Binded socket is null" << std::endl;
      return false;
    }
    rp::net::SocketAddress socket(ipStr, port);
    return IS_OK(_binded_socket->connect(socket));
  }
```
