#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <dynamixel_host_layer.h>

using namespace HostCommLayer;


class MockI2c : public I2cInterface
{
public:
  MOCK_METHOD2(write, mraa::Result(const uint8_t* data, int length));
  MOCK_METHOD2(read, int(uint8_t* data, int length));
};

TEST(DynamixelHostLayerTests, MockTest)
{
  MockI2c test;
  uint8_t data[4] = {0, 1, 2, 3};
  EXPECT_CALL(test, write(data, 4))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  test.write(data, 4);
}

int main(int argc, char **argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
