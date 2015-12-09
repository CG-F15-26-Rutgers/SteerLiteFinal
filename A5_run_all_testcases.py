# Move this to the build/bin folder before running.
# Make sure that student_grader_global and CG_F_15.json are
# moved to the bin/ folder before running.

from os import system # system() is used to execute shell commands.

testcases = ["plane_egress", "plane_ingress", "crowd_crossing",
             "office-complex", "hallway-four-way-rounded-roundabout",
             "bottleneck-squeeze", "doorway-two-way", "double-squeeze",
			 "wall-squeeze", "hallway-two-way", "maze"]

command = "./steersim -testcase REPLACE -ai sfAI -storesimulation REPLACE-26.rec"

for testcase in testcases:
    system(command.replace("REPLACE", testcase))

system("./student_grader_global 26")
