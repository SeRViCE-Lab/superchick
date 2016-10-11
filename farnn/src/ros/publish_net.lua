--[[
This publishes the network as a ros topic;
The topic can be retrieved in labview using ROS for LabVIEW

Author: Olalekan Ogunmolu
		October 2016
]]

ros = require 'ros'
ros.init('soft_robot')

spinner = ros.AsyncSpinner()
spinner.start()

nh = ros.NodeHandle()
neural_weights = ros.MsgSpec('std_msgs/Float32MultiArray')

pub = nh:advertise("neural_net", neural_weights, 100, false)
ros.spin()

msg = ros.Message(neural_weights)

if not ros.ok() then
	return
end

if pub:getNumSubscribers() == 0 then
	print('waiting for subscriber()')
else
	msg.data = neunet
	pub:publish(msg)
end

sys.sleep(0.01)
ros.spin()

