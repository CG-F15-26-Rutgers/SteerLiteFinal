from os import system

testcases = ["plane_egress", "plane_ingress", "crowd_crossing",
			 "office-complex", "hallway-four-way-rounded-roundabout",
			 "bottleneck-squeeze.xml", "doorway-two-way",
			 "double-squeeze", "wall-squeeze", "hallway-two-way", "maze"]
command = "./steersim -testcase REPLACE -ai sfAI -storesimulation REPLACE-26.rec"

for testcase in testcases:
	system(command.replace("REPLACE", testcase))

system("./student_grader_global 26")
