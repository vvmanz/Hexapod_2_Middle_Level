#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include "std_msgs/msg/int32_multi_array.h"
#include <rosidl_runtime_c/primitives_sequence_functions.h>


#include "stdbool.h"

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);

extern bool w5500_transport_open(struct uxrCustomTransport *transport);
extern bool w5500_transport_close(struct uxrCustomTransport *transport);
extern size_t w5500_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
extern size_t w5500_transport_read(struct uxrCustomTransport *transport,
		uint8_t *buf, size_t len, int timeout, uint8_t *err);

//----ROS state----//
uint32_t ros_deb;
uint32_t prev_ros_deb = 0;
uint8_t ros_state = 0;
uint8_t ros_timeout;
uint8_t dead_counter = 0;
uint8_t dead_threshold = 10;

//-----ROS arrays----///
//ros receive
uint32_t call;
int32_t ros_set_angles[6];
int32_t ros_set_koeffs[42];
int32_t ros_set_other[27];
volatile uint8_t ros_new_coeffs_flag = 0;

void angles_callback(const void *msgin) {
	const std_msgs__msg__Int32MultiArray *msg =
			(const std_msgs__msg__Int32MultiArray*) msgin;

	if (msg->data.size < 6)
		return;

	memcpy(ros_set_angles, msg->data.data, 6 * sizeof(int32_t));

	set_angle1_1 = ros_set_angles[0];
	set_angle1_2 = ros_set_angles[1];
	set_angle2_1 = ros_set_angles[2];
	set_angle2_2 = ros_set_angles[3];
	set_angle3_1 = ros_set_angles[4];
	set_angle3_2 = ros_set_angles[5];
}

void other_callback(const void *msgin) {
	const std_msgs__msg__Int32MultiArray *msg =
			(const std_msgs__msg__Int32MultiArray*) msgin;

	if (msg->data.size < 27)
		return;

	memcpy(ros_set_other, msg->data.data, 27 * sizeof(int32_t));

	int base = 0;
	obstacle1_1 = ros_set_other[base + 0];
	obstacle1_2 = ros_set_other[base + 1];
	phase1_1 = ros_set_other[base + 2];
	phase1_2 = ros_set_other[base + 3];
	set_zero = ros_set_other[base + 4];
	system_type1_1 = ros_set_other[base + 5];
	system_type1_2 = ros_set_other[base + 6];
	speed_limit1_1 = ros_set_other[base + 7];
	speed_limit1_2 = ros_set_other[base + 8];

	base = 9;
	obstacle2_1 = ros_set_other[base + 0];
	obstacle2_2 = ros_set_other[base + 1];
	phase2_1 = ros_set_other[base + 2];
	phase2_2 = ros_set_other[base + 3];
	system_type2_1 = ros_set_other[base + 5];
	system_type2_2 = ros_set_other[base + 6];
	speed_limit2_1 = ros_set_other[base + 7];
	speed_limit2_2 = ros_set_other[base + 8];

	base = 18;
	obstacle3_1 = ros_set_other[base + 0];
	obstacle3_2 = ros_set_other[base + 1];
	phase3_1 = ros_set_other[base + 2];
	phase3_2 = ros_set_other[base + 3];
	system_type3_1 = ros_set_other[base + 5];
	system_type3_2 = ros_set_other[base + 6];
	speed_limit3_1 = ros_set_other[base + 7];
	speed_limit3_2 = ros_set_other[base + 8];
}

void koeffs_callback(const void *msgin) {
	const std_msgs__msg__Int32MultiArray *msg =
			(const std_msgs__msg__Int32MultiArray*) msgin;

	if (msg->data.size < 42)
		return;

	call++;

	const int32_t *d = msg->data.data;

	for (int m = 0; m < 6; ++m) {
		int base = m * 7;
		int bridge = m / 2 + 1;
		int slot = (m % 2) + 1;

		if (bridge == 1 && slot == 1) {
			kp_angle_1_1 = d[base + 0];
			ki_angle_1_1 = d[base + 1];
			kd_angle_1_1 = d[base + 2];
			kp_speed_1_1 = d[base + 3];
			ki_speed_1_1 = d[base + 4];
			kp_curr_1_1 = d[base + 5];
			ki_curr_1_1 = d[base + 6];
		} else if (bridge == 1 && slot == 2) {
			kp_angle_1_2 = d[base + 0];
			ki_angle_1_2 = d[base + 1];
			kd_angle_1_2 = d[base + 2];
			kp_speed_1_2 = d[base + 3];
			ki_speed_1_2 = d[base + 4];
			kp_curr_1_2 = d[base + 5];
			ki_curr_1_2 = d[base + 6];
		} else if (bridge == 2 && slot == 1) {
			kp_angle_2_1 = d[base + 0];
			ki_angle_2_1 = d[base + 1];
			kd_angle_2_1 = d[base + 2];
			kp_speed_2_1 = d[base + 3];
			ki_speed_2_1 = d[base + 4];
			kp_curr_2_1 = d[base + 5];
			ki_curr_2_1 = d[base + 6];
		} else if (bridge == 2 && slot == 2) {
			kp_angle_2_2 = d[base + 0];
			ki_angle_2_2 = d[base + 1];
			kd_angle_2_2 = d[base + 2];
			kp_speed_2_2 = d[base + 3];
			ki_speed_2_2 = d[base + 4];
			kp_curr_2_2 = d[base + 5];
			ki_curr_2_2 = d[base + 6];
		} else if (bridge == 3 && slot == 1) {
			kp_angle_3_1 = d[base + 0];
			ki_angle_3_1 = d[base + 1];
			kd_angle_3_1 = d[base + 2];
			kp_speed_3_1 = d[base + 3];
			ki_speed_3_1 = d[base + 4];
			kp_curr_3_1 = d[base + 5];
			ki_curr_3_1 = d[base + 6];
		} else { // bridge == 3 && slot == 2
			kp_angle_3_2 = d[base + 0];
			ki_angle_3_2 = d[base + 1];
			kd_angle_3_2 = d[base + 2];
			kp_speed_3_2 = d[base + 3];
			ki_speed_3_2 = d[base + 4];
			kp_curr_3_2 = d[base + 5];
			ki_curr_3_2 = d[base + 6];
		}
	}

	ros_new_coeffs_flag = 1;
}

void StartRosTask(void *argument) {
	rmw_uros_set_custom_transport(
	false, NULL, w5500_transport_open, w5500_transport_close,
			w5500_transport_write, w5500_transport_read);

	rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
	allocator.allocate = microros_allocate;
	allocator.deallocate = microros_deallocate;
	allocator.reallocate = microros_reallocate;
	allocator.zero_allocate = microros_zero_allocate;
	rcutils_set_default_allocator(&allocator);

	rclc_support_t support;
	rcl_node_t node;
	rclc_executor_t executor;

	rcl_publisher_t pub_vector;
	rcl_subscription_t sub_angles;
	rcl_subscription_t sub_other;
	rcl_subscription_t sub_koeffs;

	std_msgs__msg__Int32MultiArray msg_vector;
	std_msgs__msg__Int32MultiArray msg_angles;
	std_msgs__msg__Int32MultiArray msg_other;
	std_msgs__msg__Int32MultiArray msg_koeffs;

	static bool msgs_inited = false;

	ros_state = 0;
	dead_counter = 0;

	uint32_t ping_div = 0;

	for (;;) {
		switch (ros_state) {
		case 0: // WAIT AGENT
			if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
				ros_state = 1;
			osDelay(100);
			break;

		case 1: // INIT
			rclc_support_init(&support, 0, NULL, &allocator);
			rclc_node_init_default(&node, "STM32F446RET6", "", &support);

			rclc_publisher_init_default(&pub_vector, &node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
					"vector");

			if (!msgs_inited) {
				std_msgs__msg__Int32MultiArray__init(&msg_vector);
				rosidl_runtime_c__int32__Sequence__init(&msg_vector.data, 18);

				std_msgs__msg__Int32MultiArray__init(&msg_angles);
				msg_angles.data.data = ros_set_angles;
				msg_angles.data.capacity = 6;

				std_msgs__msg__Int32MultiArray__init(&msg_other);
				msg_other.data.data = ros_set_other;
				msg_other.data.capacity = 27;

				std_msgs__msg__Int32MultiArray__init(&msg_koeffs);
				msg_koeffs.data.data = ros_set_koeffs;
				msg_koeffs.data.capacity = 42;

				msgs_inited = true;
			}

			rclc_subscription_init_default(&sub_angles, &node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
					"set_angles");

			rclc_subscription_init_default(&sub_other, &node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
					"set_other");

			rclc_subscription_init_default(&sub_koeffs, &node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
					"set_koeffs");

			rclc_executor_init(&executor, &support.context, 3, &allocator);
			rclc_executor_add_subscription(&executor, &sub_angles, &msg_angles,
					angles_callback, ON_NEW_DATA);
			rclc_executor_add_subscription(&executor, &sub_other, &msg_other,
					other_callback, ON_NEW_DATA);
			rclc_executor_add_subscription(&executor, &sub_koeffs, &msg_koeffs,
					koeffs_callback, ON_NEW_DATA);

			dead_counter = 0;
			ping_div = 0;
			ros_state = 2;
			break;

		case 2: // RUN
		{
			int32_t data[18] = { angle1_1, speed1_1, current1_1, angle1_2,
					speed1_2, current1_2, angle2_1, speed2_1, current2_1,
					angle2_2, speed2_2, current2_2, angle3_1, speed3_1,
					current3_1, angle3_2, speed3_2, current3_2 };

			memcpy(msg_vector.data.data, data, sizeof(data));
			msg_vector.data.size = 18;

			(void)rcl_publish(&pub_vector, &msg_vector, NULL);
			rclc_executor_spin_some(&executor, 0);

			if (++ping_div >= 50) {
				ping_div = 0;
				if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK)
					dead_counter++;
				else
					dead_counter = 0;
			}

			if (dead_counter > dead_threshold) {
				rclc_executor_fini(&executor);
				(void)rcl_node_fini(&node);
				rclc_support_fini(&support);
				ros_state = 0;
			}

			osDelay(1);
			break;
		}
		}
	}
}

