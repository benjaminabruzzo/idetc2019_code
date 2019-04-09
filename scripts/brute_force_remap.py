#!/bin/python


filename_in = '/home/benjamin/ros/src/metahast/hast/launch/kobuki/brute_force_remap.launch'
filename_out = '/home/benjamin/ros/src/metahast/hast/launch/kobuki/brute_force_remap.remap'
# with open() as f:
#     lines = f.readlines()


with open(filename_in, "r") as ins:
    array = []
    for line in ins:
        array.append(line.rstrip('\n'))

with open(filename_out, 'w') as the_file:
	for line in array:
		remap_from = ('<remap from="{0}" ').format(line)
		fill_to = ('{text: >{fill}}').format(text = 'to', fill='20')
		jutify_left = ('{text: <{fill}}').format(text = remap_from, fill='80')
		remap_to = ('to="/$(arg ugv_ns){0}" />').format(line)
		newline = jutify_left + remap_to
		the_file.write(newline +'\n')




		# # <remap from="/move_base/DWAPlannerROS/cost_cloud" to="/$(arg ugv_ns)/move_base/DWAPlannerROS/cost_cloud" />
		# remap_from = ('<remap from="{message: <{fill}}" ').format(message=line, fill='70')
		# remap_to = ('to="/$(arg ugv_ns){0}" />').format(line)
		
		# remap_to = ('to="/$(arg ugv_ns){0}" />').format(line)
		# newline = remap_from + remap_to
		# # print(newline)
		# the_file.write(newline +'\n')
