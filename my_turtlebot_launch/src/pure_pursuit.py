#PythonDraw.py
import turtle as tu
tu.setup(650,350,200,200)
tu.penup()
tu.fd(-250)
tu.pendown()
tu.pensize(25)
tu.seth(-40)
for i in range(4):
    tu.pencolor("yellow")
    tu.circle(40,80)
    tu.pencolor("gold")
    tu.circle(-40,80)
tu.circle(40,80/2)
tu.fd(40)
tu.circle(32,180)
tu.fd(40*2/3)
tu.done()