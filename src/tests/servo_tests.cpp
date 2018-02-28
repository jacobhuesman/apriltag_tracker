#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <dynamixel_host_layer.h>
#include <comm_layer_defs.h>

using namespace HostCommLayer;


class MockI2c : public I2cInterface
{
public:
  MOCK_METHOD2(write, mraa::Result(const uint8_t* data, int length));
  MOCK_METHOD2(read, int(uint8_t* data, int length));
};

TEST(DynamixelHostLayerTests, MockWriteTest)
{
  MockI2c test;
  uint8_t data[4] = {0, 1, 2, 3};
  EXPECT_CALL(test, write(data, 4))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  test.write(data, 4);
}

TEST(DynamixelHostLayerTests, ReadTest)
{
  MockI2c test;
  uint8_t data[4] = {0, 1, 2, 3};
  EXPECT_CALL(test, read(data, 4))
      .Times(1)
      .WillOnce(testing::Return(4));
  test.read(data, 4);
}

TEST(DynamixelHostLayerTests, Constructor)
{
  using testing::_;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, 4)).Times(1).WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c,  read(_, 4)).Times(1).WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  servo.writeI2c(&message);
  servo.readI2c(&message);
  geometry_msgs::TransformStamped tf = servo.getTransformMsg();
  ASSERT_STREQ(tf.header.frame_id.c_str(), "servo_base_link");
  ASSERT_STREQ(tf.child_frame_id.c_str(), "servo_joint");
}

TEST(DynamixelHostLayerTests, ComputeChecksum)
{
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_POSITION;
  message.ucl.data = 12;
  message.ucl.checksum = Dynamixel::computeChecksum(message);

  ASSERT_EQ(message.ucl.checksum, 0xDD);
}

int main(int argc, char **argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
