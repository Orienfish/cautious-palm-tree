# cautious-palm-tree
Using modbus tcp and modbus rtu protocol on linux to control and read data from ur3

In this project, you can find 2 files: modbus_tcp.c and modbus_rtu.c.

modbus_tcp.c - Using modbus tcp protocol to read real-time information from ur3.
               Main Function：
               a.using modbus tcp to read ur3 TCP's position[x,y,z,rx,ry,rz], in the base coordinate system.
               b.read ur3's 6 joints' angle[base,shoulder,elbow,wrist1,wrist2,wrist3]
 
               Note: [x,y,z,rx,ry,rz] are signed, while [base,shoulder,elbow,wrist1,wrist2,wrist3] are unsigned.
               So you may consider limit the scope of each joint to make sure that their sign is certain, either >0 or <0.
               
               Note: You have to make installation settings on the teaching pendant of ur3. In this case, I use a wire to connect the PC                  and the ur3.
              
modbus_rtu.c - Using modbus rtu protocol to control 2-finger gripper
               Main Function：
               a.using modbus rtu to activate gripper
               b.close the gripper with full force and full speed
               c.open the gripper with full force and full speed
 
                Note: some of the instruction is different from the Instruction Manual provided by Universal Robot.
                I don't know why they are different, but I get these from real robot test.
                
                Note: In this case, I use a USB-RS485 converter to connect the PC and the gripper.
