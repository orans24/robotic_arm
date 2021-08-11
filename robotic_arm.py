import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.integrate import odeint
from matplotlib import animation
import math
import scipy
import pandas as pd
import sympy
from sys import exit #i used the moudle just for stopping the run


#the input for Q2
x_ee = [float(550), float(600),float(30)]# Current position of the end-effector
x_cube = [float(-600), float(-600),float(0)]# Position of the cube on the table
dpush = float(100)# Push distance

#Input for Q3,Q4,Q5
xanimatestart= [float(-1),float(-1)]
xanimategoal= [float(450),float(-450)]

#constant parameters
cubesize=[50.0,50.0,50.0]
cubeang = np.arctan2(x_cube[1], x_cube[0])
col = 10
ab1c = np.array([370, 260])
ab2c = np.array([-330, 375])
ab3c = np.array([-350, -175])
ab1r = 90.0
ab2r = 25.0
ab3r = 65.0


# if you use pycharm make sure:
# setting-->python scientific-->uncheck show plots in tool window for Backend TkAgg
#the code has ben Written and tested in Pycharm
class robot_arm():
    #robot links l0 is l1 and so on...
    l0 = 160.0
    l1 = 340.0
    l2 = 380.0
    l3 = 130.0
    rmax = l1 + l2 + l3

    #the UI function to manage the runing include:
    #Run Q1,Q2,Q3,Q4,Q5
    #checking edge case for Q2
    #choosing between random numbers or the assigned numbers for Q2 running
    def start(self):
        global x_ee,x_cube
        q=['1','2','3','4','5']
        qnum = str(input('choose the question to check 1-5:'))
        while qnum not in q:
            qnum = str(input('choose between 1 to 5'))
        qnum =int(qnum)
        if qnum == 1:
            robot.taskspace()
        elif qnum == 2:
            rr=['0','1']
            rg=str(input('run with random numbers or with the given one on top? type 1 for random or 0 for given'))
            while rg not in rr:
                rg = str(input('choose 1 for random or 0 for given'))
            rg=int(rg)
            if rg == 0:
                ans=['y','n']
                while not robot.collisionchecker(x_ee[0], x_ee[1], x_ee[2]):
                    print('the position of the end-effector in collision or too far !! change his values')
                    an=str(input('would you like to choose another position for him? y/n'))
                    while an not in ans:
                        an = str(input('would you like to choose another position for him? y/n'))
                    if an =='n':
                        exit()
                    if an=='y':
                        x_ee=[float(input('x')),float(input('y')),float(input('z'))]

                while not robot.collisionchecker(x_cube[0], x_cube[1], (x_cube[2] + cubesize[2] / 2)):
                    print('the position of the cube in collision or cant reach !!')
                    an = str(input('would you like to choose another position for him? y/n'))
                    while an not in ans:
                        an = str(input('would you like to choose another position for him? y/n'))
                    if an == 'n':
                        exit()
                    if an == 'y':
                        x_cube = [float(input('x')), float(input('y')),float(input('z'))]

                while x_cube[2]<0 or  x_cube[2]>0 :
                    print('the cube is not on the plane')
                    an = str(input('would you like to choose another position for him? y/n'))
                    while an not in ans:
                        an = str(input('would you like to choose another position for him? y/n'))
                    if an == 'n':
                        exit()
                    if an == 'y':
                        x_cube[2]= float(input('z'))

                robot.xyzgraph()

            elif rg == 1:
                robot.randomtest()

        elif qnum==3:
            robot.animateplot()

        elif qnum == 4:
            robot.dynamic(control=False)

        elif qnum == 5:
            robot.dynamic(control=True)
            robot.animateplot()

    #main code of Q1 getting the xyz of ee taskspace(Q1)
    def taskspace(self):
        xcord = []
        ycord = []
        zcord = []
        while len(xcord)<25000:
            q=np.random.random((4,))*(np.array([2*np.pi,2*np.pi,2*np.pi,2*np.pi])).reshape((4,))+np.array([-np.pi,-np.pi,-np.pi,-np.pi]).reshape((4,))
            cord = robot.direct_kinematics(q)
            if robot.collisionchecker(cord[0],cord[1],cord[2]):
                xcord.append(cord[0])
                ycord.append(cord[1])
                zcord.append(cord[2])

        ax = plt.axes(projection='3d')
        ax.scatter(xcord, ycord, zcord)
        ax.set_xlim3d(-850, 850)
        ax.set_ylim3d(-850, 850)
        ax.set_zlim3d(-200, 1000)
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
        plt.show()

    #direct kinematics for ee(Q1,Q2,Q3) get ee cordinate for given angles of the joints
    def direct_kinematics(self,q):
        r = self.l1 * np.cos(q[1]) + self.l3 * np.cos(q[1] + q[2] + q[3]) + self.l2 * np.cos(q[1] + q[2])
        x = r * np.cos(q[0])
        y = r * np.sin(q[0])
        z = self.l0 + self.l1 * np.sin(q[1]) + self.l2 * np.sin(q[1] + q[2]) + self.l3 * np.sin(q[1] + q[2] + q[3])

        return x,y,z

    #random numbers for Q2
    def randomtest(self):
        global x_ee, x_cube, dpush
        x_ee = [np.random.uniform(-750, 750), np.random.uniform(-750, 750),np.random.uniform(0, 750)]  # Current position of the end-effector
        x_cube = [np.random.uniform(-750, 750),np.random.uniform(-750, 750),float(0)]  # Position of the cube on the table
        dpush = np.random.uniform(10,100) # Push distance
        while not robot.collisionchecker(x_ee[0], x_ee[1], x_ee[2]):
            print('the position of the end-effector in collision or too far !!'+str(x_ee))
            x_ee = [np.random.uniform(-750, 750), np.random.uniform(-750, 750),np.random.uniform(0, 750)]  # Current position of the end-effector
        while not robot.collisionchecker(x_cube[0], x_cube[1], (x_cube[2] + cubesize[2] / 2)):
            print('the position of the cube in collision or cant reach !!'+str(x_cube))
            x_cube = [np.random.uniform(-750, 750), np.random.uniform(-750, 750),float(0)]  # Position of the cube on the table
        robot.xyzgraph()

    #main code of Q2 plotting the graph with the path(Q2)
    def xyzgraph(self):
        fig = plt.figure(figsize=(10, 10))
        ax = plt.axes(projection='3d')
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
        x1, y1, z1 = robot.makeccylindrical(ab1c, ab1r,1000)
        x2, y2, z2 = robot.makeccylindrical(ab2c, ab2r,1000)
        x3, y3, z3 = robot.makeccylindrical(ab3c, ab3r,1000)
        ax.plot_surface(x1, y1, z1,label='obstacle 1' )
        ax.plot_surface(x2, y2, z2,label='obstacle 2' )
        ax.plot_surface(x3, y3, z3,label='obstacle 3' )
        xs = np.array([x_ee[0],x_ee[1]])
        xg = np.array([x_cube[0]-np.cos(cubeang)*cubesize[0]/2,x_cube[1]-cubesize[0]/2*np.sin(cubeang)])
        X1=robot.rrtpath(xs,xg)
        X2 = np.array(robot.pushingcube(np.copy(X1[-1])))
        zfix=np.linspace(x_ee[2],cubesize[2]/2,len(X1)) #to the center of the cube
        ax.plot3D(X1[:,0], X1[:,1],zfix,color='blue',label='Catcher movement')
        if len(X2)==0:
            ax.text2D(0.05, 0.95, "cant push the cube!!",color='red', transform=ax.transAxes)
        else:
            zpush=np.outer(cubesize[2]/2, np.ones(len(X2)))
            ax.plot3D(X2[:,0],X2[:,1],zpush[0],color='red',label='Catcher pushing')

        ax.text2D(0.4, 0.95,'x_ee '+str(x_ee), color='black', transform=ax.transAxes)
        ax.text2D(0.4, 0.92,'x_cube ' + str(x_cube), color='black', transform=ax.transAxes)
        ax.set_xlim3d(-850, 850)
        ax.set_ylim3d(-850, 850)
        ax.set_zlim3d(0, 1000)
        plt.show()

    #RRT path planing between two points(Q2,Q3)
    def rrtpath(self,xs,xg):
        x=np.copy(xs)
        rmove=np.linalg.norm(x - xg)/20
        X1=[]
        while np.linalg.norm(x - xg)>0.01 or abs(robot.planchecker(x,xg))>0.05:
            X1.append(x)
            checkingnodes=[]
            dnodes=[]
            moveang=np.linspace(-np.pi,np.pi,360)
            for teta in moveang:
                xmove=rmove*np.cos(teta)
                ymove=rmove*np.sin(teta)
                rnx = np.array([x[0] + xmove, x[1] + ymove])
                if robot.collisionchecker(rnx[0],rnx[1],25):
                    checkingnodes.append(rnx)
                    dnodes.append(np.linalg.norm(rnx-xg))
            bestnode=checkingnodes[0]
            indexbn=0
            for i in range(1,len(checkingnodes)):
                if dnodes[i]<dnodes[indexbn]:
                    bestnode=checkingnodes[i]
                    indexbn=i
            x=bestnode
            rmove = np.linalg.norm(x - xg) /20

        X1.append(xg)
        X1=np.array(X1)
        return X1

    #create cordinate arrays for cylindrical plotting the obstacle(Q2,Q3)
    def makeccylindrical(self,cen,r,h):
        u = np.linspace(-np.pi,np.pi, 20)  # divide the circle into 20 equal parts
        h = np.linspace(0,h, 5)  # divide the height into 5 parts
        x = np.outer(cen[0]+r*np.cos(u), np.ones(len(h)))  # x value repeate
        y = np.outer(cen[1]+r*np.sin(u), np.ones(len(h)))  # y value repeated
        z = np.outer(np.ones(len(u)), h)  # x,y corresponding height
        return x,y,z

    #collision check(Q2) return false for collision
    def collisionchecker(self, x, y, z):
        if math.sqrt(np.square(x) + np.square(y)+ np.square(z))>self.rmax:
            return False
        if math.sqrt(np.square(ab1c[0] - x) + np.square(ab1c[1] - y)) < ab1r + col:
            return False
        elif math.sqrt(np.square(ab2c[0] - x) + np.square(ab2c[1] - y)) < ab2r + col:
            return False
        elif math.sqrt(np.square(ab3c[0] - x) + np.square(ab3c[1] - y)) < ab3r + col:
            return False
        elif (x > ab1c[0]) and (x * (ab1c[1] - ab1r) / ab1c[0] < y < x * (ab1c[1] + ab1r) / ab1c[0]):
            return False
        elif (x < ab2c[0]) and (x * (ab2c[1] - ab2r) / ab2c[0] < y < x * (ab2c[1] + ab2r) / ab2c[0]):
            return False
        elif (x < ab3c[0]) and (x * (ab3c[1] - ab3r) / ab3c[0] < y < x * (ab3c[1] + ab3r) / ab3c[0]):
            return False
        elif z < col:
            return False

        else:
            return True

    #check the angle between two location(Q2)
    def planchecker(self,ee,cube):
        ang1 =np.arctan2(ee[0],ee[1])
        ang2 =np.arctan2(cube[0],cube[1])
        angle =ang1-ang2
        return angle

    #get the cordinate while pushing the cube and check edge case for stopping(Q2)
    def pushingcube(self,p):
        X=[]
        obs1 = np.array([ab1c[0], ab1c[1]])
        obs2 = np.array([ab2c[0], ab2c[1]])
        obs3 = np.array([ab3c[0], ab3c[1]])
        cubecenter=cubesize[2]/2
        rpushmax=math.sqrt(np.square(self.rmax) - np.square(self.l0-cubecenter))
        p1=np.copy(p)
        rstart = math.sqrt(np.square(p1[0]) + np.square(p1[1]))
        r=rstart
        alph=np.arctan2(p1[1],p1[0])
        d1= np.linalg.norm(p1 - obs1)
        d2 = np.linalg.norm(p1 - obs2)
        d3 = np.linalg.norm(p1 - obs3)
        dmin = np.min(np.array([d1, d2, d3]))
        if dmin==d1:
            obclose=obs1
            abrclose=ab1r
        if dmin == d2:
            obclose = obs2
            abrclose = ab2r
        if dmin == d3:
            obclose = obs3
            abrclose = ab3r
        if abs(robot.planchecker(p1,obclose))<0.1:#front
            dmin-=50
        else:
            dmin-=25#sides

        while (d1 > ab1r + col) and (d2 > ab2r + col) and (d3 > ab3r + col) and (dmin>abrclose+col) and (r <= rpushmax) and (r - rstart <= dpush) :
            r+=0.005
            p1=np.array([r*np.cos(alph),r*np.sin(alph)])
            X.append(np.copy(p1))
            d1 = np.linalg.norm(p1 - obs1)
            d2 = np.linalg.norm(p1 - obs2)
            d3 = np.linalg.norm(p1 - obs3)
            dmin = np.min(np.array([d1, d2, d3]))
            if dmin == d1:
                obclose = obs1
                abrclose = ab1r
            if dmin == d2:
                obclose = obs2
                abrclose = ab2r
            if dmin == d3:
                obclose = obs3
                abrclose = ab3r
            if abs(robot.planchecker(p1,obclose))<0.1:#front
                dmin -= 50
            else:
                dmin -= 25  # sides
        return X

    #main code of Q3 create animate for simulation of moving arm robot between points(Q3)
    def animateplot(self):
        Q=[]
        e = np.empty((0, 3))
        xg = np.copy(xanimategoal)
        x = np.copy(xanimatestart)
        X1 = robot.rrtpath(x,xg)
        zfix=np.linspace(float(700),float(170),len(X1))
        zfix=np.reshape(zfix,(-1,1))
        X1 = np.array(X1)
        X1= np.hstack((X1,zfix))
        for xx in X1:
            Q.append(robot.inverse_kinematics(xx,float(0)))
        for q in Q:
            ee = np.asarray(self.direct_kinematics(q))
            ee = ee.reshape(1, 3)
            e = np.append(e, ee.reshape(1, -1), axis=0)
        Q=np.array(Q)
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        ax = plt.axes(projection='3d')
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
        x0, y0, z0 = robot.makeccylindrical([0, 0], 50, 130)  # 130 instead of 160 just for good visual in 3d
        x1, y1, z1 = robot.makeccylindrical(ab1c, ab1r, 1000)
        x2, y2, z2 = robot.makeccylindrical(ab2c, ab2r, 1000)
        x3, y3, z3 = robot.makeccylindrical(ab3c, ab3r, 1000)
        ax.plot_surface(x0, y0, z0, color='grey', label='Base')
        ax.plot_surface(x1, y1, z1, label='obstacle 1')
        ax.plot_surface(x2, y2, z2, label='obstacle 2')
        ax.plot_surface(x3, y3, z3, label='obstacle 3')
        ax.set_xlim3d(-850, 850)
        ax.set_ylim3d(-850, 850)
        ax.set_zlim3d(0, 1000)
        X = self.joins_direct_kinematics(Q[0])
        ee = np.asarray(self.direct_kinematics(Q[0]))
        ee = ee.reshape(1, 3)
        line,=ax.plot(X[:,0], X[:, 1],X[:, 2], '-ok',color='grey',linewidth=5,markersize=8)
        endeff,=ax.plot(ee[0,0], ee[0,1],ee[0,2],'-ok',color='blue', markersize=10)  # Plot end-effector - red marker
        track,=ax.plot(e[0:1, 0], e[0:1, 1],e[0:1,2],'--b')

        def updateanimate(i):
            q=Q[i,:]
            X=self.joins_direct_kinematics(q)
            ee=np.asarray(self.direct_kinematics(q))
            ee=ee.reshape(1,3)
            X=np.vstack((X,ee))
            line.set_data(X[:,0], X[:, 1])
            line.set_3d_properties(X[:, 2])
            endeff.set_data(ee[0,0], ee[0,1])
            endeff.set_3d_properties(ee[0,2])
            track.set_data(e[0:i, 0], e[0:i, 1])
            track.set_3d_properties(e[0:i,2])
            return line,endeff,track

        ani = animation.FuncAnimation(fig,updateanimate,frames=int(Q.shape[0]),interval=20,blit=False)
        #ani.save('simulation.gif', writer='pillow')
        plt.show()
        ani._start()

    #inverse kinematics(Q3)get angles of the joints for given ee cordinate and angle
    def inverse_kinematics(self,p,teta234):
        r= math.sqrt(np.square(p[0]) + np.square(p[1]))
        r3=r-self.l3*np.cos(teta234)
        z3=p[2]-self.l3*np.sin(teta234)
        D=((r3**2 + (z3-self.l0)**2 - self.l2**2 - self.l1**2) / (2*self.l1*self.l2))
        q=[]
        teta1=np.arctan2(p[1],p[0])
        teta3= np.arctan2(np.sqrt(np.abs(1 - D ** 2)), D)
        teta2=np.arctan2(z3-self.l0,r3) - np.arctan2(self.l2*np.sin(teta3),self.l1 + self.l2 * np.cos(teta3))
        teta4=teta234-teta2-teta3
        q.append(teta1)
        q.append(teta2)
        q.append(teta3)
        q.append(teta4)
        q=np.array(q)
        return q

    #direct kinematics(Q3) get joints cordinate for given angles of the joints
    def joins_direct_kinematics(self, q):
        r1= self.l1 * np.cos(q[1])
        x1 = r1 * np.cos(q[0])
        y1 = r1 * np.sin(q[0])
        z1 = self.l0 + self.l1 * np.sin(q[1])
        r2 = self.l1 * np.cos(q[1]) + self.l2 * np.cos(q[1] + q[2])
        x2 = r2 * np.cos(q[0])
        y2 = r2 * np.sin(q[0])
        z2 = self.l0 + self.l1 * np.sin(q[1]) + self.l2 * np.sin(q[1] + q[2])
        return np.array([[0,0,160],[x1,y1,z1],[x2,y2,z2]])

    #dynamic model(Q4,Q5) simulate moving of robot arm with/without control and friction.
    #simulate the path between two points consider gravity , kinetic ,friction and potential energy
    #Plot wanted vs actually
    def dynamic(self,control):
        Q=[]
        Tf = 20
        e = np.empty((0, 3))
        xg = np.copy(xanimategoal)
        x = np.copy(xanimatestart)
        X1 = robot.rrtpath(x,xg)
        zfix=np.linspace(float(700),float(170),len(X1))
        zfix=np.reshape(zfix,(-1,1))
        X1 = np.array(X1)
        X1= np.hstack((X1,zfix))
        for xx in X1:
            Q.append(robot.inverse_kinematics(xx,float(0)))
        t = np.linspace(0, Tf,len(Q))
        x0=Q[0]
        x0=np.append(x0,np.zeros((4)))
        xg=Q[len(Q)-1]
        xg=np.append(xg,np.zeros((4)))

        def Mmatrix(q):
            M = np.zeros((4, 4))
            M[0,0] = (self.l0**3)/12+(self.l1**3*np.cos(q[1])**2)/4\
                     +(self.l2**3*np.cos(q[1]+q[2])**2)/4 + \
                     (self.l3**3*np.cos(q[1]+q[2]+q[3])**2)/4
            M[1, 1] = (self.l1 ** 3) / 3 + (self.l2 ** 3) / 4 + (self.l3 ** 3) / 4
            M[1, 2] = (self.l2 ** 3) / 4 + (self.l3 ** 3) / 4
            M[1, 3] = (self.l3 ** 3) / 4
            M[2, 1] = (self.l2 ** 3) / 4 + (self.l3 ** 3) / 4
            M[2, 2] = (self.l2 ** 3) / 3 + (self.l3 ** 3) / 4
            M[2, 3] = (self.l3 ** 3) / 4
            M[3, 1] = -(self.l3 ** 3) / 4
            M[3, 2] = (self.l3 ** 3) / 4
            M[3, 3] = (self.l3 ** 3) / 3
            return M

        def Cmatrix(q,dq):
            C = np.zeros((4, 4))
            C[0,1]=-(dq[0]*self.l1**3*np.sin(2*q[1]))/4\
                   -(dq[0] * self.l2 ** 3 * np.sin(2 *(q[2]+q[1])))/4\
                   -(dq[0] * self.l3 ** 3 * np.sin(2 *(q[3]+q[2]+q[1])))/4

            C[0,2] = -(dq[0] * self.l2 ** 3 * np.sin(2 *(q[2]+q[1])))/4\
                     -(dq[0] * self.l3 ** 3 * np.sin(2 *(q[3]+q[2]+q[1])))/4

            C[0,3] = -(dq[0] * self.l3 ** 3 * np.sin(2 *(q[3]+q[2]+q[1])))/4
            return C

        def Gmatrix(q):
            g2=9.81*(np.cos(q[1])*(self.l1**2+self.l1*self.l2+self.l3)+1/2*(self.l2**2*np.cos(q[1]+q[2])+self.l3**2*np.cos(q[1]+q[2]+q[3])))
            g3=9.81*(-np.cos(q[1]+q[2])*(1/2*(self.l2**2)+self.l2*self.l3)-1/2*(self.l3**2*np.cos(q[1]+q[2]+q[3])))
            g4=-9.81/2*(self.l3**2*np.cos(q[1]+q[2]+q[3]))
            G = np.array([0.0,g2,g3,g4])
            return G

        def model(x, t, x0, xg, Tf):
            x1 = x[:4]  # q
            x2 = x[4:]  # dq
            u=0#no control
            ft=0#no friction
            # Q5 with PD control and friction
            if control == True:
                Kp = np.diag([1500,1500,1500,1500])
                Kd = np.diag([100,100,100,100])
                u = - Kp.dot(x1-Q[int(t)*10]) - Kd.dot(x2)
                fv = np.random.normal(0.0, 5.0, (4,))
                ft = np.multiply(fv, x2)

            dx1=x2
            M=Mmatrix(x1)
            invM = np.linalg.inv(M)
            G=Gmatrix(x1)
            C = Cmatrix(x1,x2)

            dx2=invM.dot(u-C.dot(x2)-G-ft)
            dxdt = np.concatenate((dx1, dx2), axis=0)
            return dxdt

        Qdynamic = odeint(model, x0, t,args=(x0, xg, Tf,))
        e = np.empty((0, 3))
        for q in Qdynamic:
            ee = np.asarray(self.direct_kinematics(q))
            ee = ee.reshape(1, 3)
            e = np.append(e, ee.reshape(1, -1), axis=0)
        Qdynamic=np.array(Qdynamic)
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        ax = plt.axes(projection='3d')
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
        x0, y0, z0 = robot.makeccylindrical([0, 0], 50, 130)  # 130 instead of 160 just for good visual in 3d
        x1, y1, z1 = robot.makeccylindrical(ab1c, ab1r, 1000)
        x2, y2, z2 = robot.makeccylindrical(ab2c, ab2r, 1000)
        x3, y3, z3 = robot.makeccylindrical(ab3c, ab3r, 1000)
        ax.plot_surface(x0, y0, z0, color='grey', label='Base')
        ax.plot_surface(x1, y1, z1, label='obstacle 1')
        ax.plot_surface(x2, y2, z2, label='obstacle 2')
        ax.plot_surface(x3, y3, z3, label='obstacle 3')
        ax.set_xlim3d(-850, 850)
        ax.set_ylim3d(-850, 850)
        ax.set_zlim3d(0, 1000)
        X = self.joins_direct_kinematics(Qdynamic[0])
        ee = np.asarray(self.direct_kinematics(Qdynamic[0]))
        ee = ee.reshape(1, 3)
        line,=ax.plot(X[:,0], X[:, 1],X[:, 2], '-ok',color='grey',linewidth=5,markersize=8)
        endeff,=ax.plot(ee[0,0], ee[0,1],ee[0,2],'-ok',color='blue', markersize=10)  # Plot end-effector - red marker
        track,=ax.plot(e[0:1, 0], e[0:1, 1],e[0:1,2],'--b')

        def updateanimate(i):
            q=Qdynamic[i,:]
            X=self.joins_direct_kinematics(q)
            ee=np.asarray(self.direct_kinematics(q))
            ee=ee.reshape(1,3)
            X=np.vstack((X,ee))
            line.set_data(X[:,0], X[:, 1])
            line.set_3d_properties(X[:, 2])
            endeff.set_data(ee[0,0], ee[0,1])
            endeff.set_3d_properties(ee[0,2])
            track.set_data(e[0:i, 0], e[0:i, 1])
            track.set_3d_properties(e[0:i,2])
            return line,endeff,track

        ani = animation.FuncAnimation(fig,updateanimate,frames=int(Qdynamic.shape[0]),interval=20,blit=False)
        #ani.save('simulation.gif', writer='pillow')
        plt.show()
        ani._start()
        if control==True:
            f, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
            ax1.plot(t, np.rad2deg(Qdynamic[:, :4]))
            ax1.set_title('Actually Angles')
            ax1.legend(('q1', 'q2','q3','q4'))
            ax1.set_xlabel('t (sec)')
            ax1.set_ylabel('q (deg)')
            ax1.set_xlim([0, np.max(t)])
            ax2.plot(t, np.rad2deg(Q))
            ax2.set_title('wanted Angles')
            ax2.legend(('q1', 'q2','q3','q4'))
            ax2.set_xlabel('t (sec)')
            ax2.set_ylabel('q (deg)')
            ax2.set_xlim([0, np.max(t)])
            plt.show()

    #not in use!! simulate the moving arm by ploting frames one by one
    # just in case if the animate function of matplotlib not work
    # because the function demand canceling python scientific in pycharm and use Backend TkAgg
    def animate(self):
        Q=[]
        e = np.empty((0, 3))
        xg = np.copy(xanimategoal)
        x = np.copy(xanimatestart)
        X1 = robot.rrtpath(x,xg)
        zfix=np.linspace(float(700),float(170),len(X1))
        zfix=np.reshape(zfix,(-1,1))
        X1= np.hstack((X1,zfix))
        for xx in X1:
            Q.append(robot.inverse_kinematics(xx,float(np.pi/2)))
        for q in Q:
            ax = plt.axes(projection='3d')
            ax.set_xlabel('$X$')
            ax.set_ylabel('$Y$')
            ax.set_zlabel('$Z$')
            x0, y0, z0 = robot.makeccylindrical([0, 0], 50, 130)  # 130 instead of 160 just for good visual in 3d
            x1, y1, z1 = robot.makeccylindrical(ab1c, ab1r, 1000)
            x2, y2, z2 = robot.makeccylindrical(ab2c, ab2r, 1000)
            x3, y3, z3 = robot.makeccylindrical(ab3c, ab3r, 1000)
            ax.plot_surface(x0, y0, z0, color='grey', label='Base')
            ax.plot_surface(x1, y1, z1, label='obstacle 1')
            ax.plot_surface(x2, y2, z2, label='obstacle 2')
            ax.plot_surface(x3, y3, z3, label='obstacle 3')
            ax.set_xlim3d(-850, 850)
            ax.set_ylim3d(-850, 850)
            ax.set_zlim3d(0, 1000)

            X=self.joins_direct_kinematics(q)
            ee=np.asarray(self.direct_kinematics(q))
            ee=ee.reshape(1,3)
            if not robot.collisionchecker(ee[0,0],ee[0,1],ee[0,2]):
                continue
            X=np.vstack((X,ee))
            ax.plot(X[:,0], X[:, 1],X[:, 2], '-ok',color='grey',linewidth=5,markersize=10)
            ax.plot(ee[0,0], ee[0,1],ee[0,2],'-ok',color='red',linewidth=5 , markersize=16)  # Plot end-effector - red marker
            e = np.append(e, ee.reshape(1, -1), axis=0)
            ax.plot(e[:, 0], e[:, 1],e[:,2],'--b')
            plt.show()
            plt.pause(0.1)
            ax.cla()



robot = robot_arm()
robot.start()




