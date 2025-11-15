<p align="center">
  <h1 align="center">PolyU AAE1001 Project Group 5 Readme (Five Egg and Toast)</h1>
</p>

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#1-group-members-and-roles">Group Members and Roles</a>
    </li>
    <li>
      <a href="#2-introduction">Introduction</a>
    </li>
    <li>
      <a href="#3-task-1">Task 1</a>
    </li>
     <li>
      <a href="#4-task-2">Task 2</a>
    </li>
    <li>
      <a href="#5-task-3">Task 3</a>
    </li>
    <li>
      <a href="#6-additional-task-1">Additional Task 1</a>
    </li>
    <li>
      <a href="#7-additional-task-2">Additional Task 2</a>
    </li>
    <li>
      <a href="#8-additional-task-3">Additional Task 3</a>
    </li>
     <li>
      <a href="#9-group-reflections">Group Reflections</a>
    </li>
    <li>
      <a href="#10-presentation-files">Presentation Files</a>
    </li>
    <li>
      <a href="#11-report">Report</a>
    </li>
  </ol>
</details>

<!-- Group Members and Roles -->
## 1. Group Members and Roles
1. Winkelhock, Yau Yue Hong Winkelhock (25034703D)
2. Rodgers, Rodgers Mawalla Maighacho (25094994D)
3. Manny, So Yee Man (25079801D)
4. August, Wang Tiancheng (25095793D)
5. Talia, Cheung Yung Ting Talia (25129573D)
6. Sharon, Chan Chin Ying (25084065D)
7. Sylvia, Lau Tsz Wing (25068927D)

<!-- Introduction -->
## 2. Introduction
### Task Path
### Path Planning

## 3. Task 1
<a href="task1.py"><strong>Task 1 Code</strong></a>
#### Description
#### Calculation Method
#### Scenario 1
#### Scenario 2
#### Scenario 3

### Bonus Part
#### Calculation with Code
#### Cost Function with Manual Calculation
#### Outputs

## 4. Task 2
<a href="task2.py"><strong>Task 2 Code</strong></a>
#### Introducing Jetstream
#### Description
The task involved creating a new cost area that can reduce the cost of the route, based on task 1's code. The aim was to find the best place within the map to set our minus-cost area (jet stream) and reduce the cost by 5%. Similarly, the area of the jet stream had to span across the map laterally and extend 5 units vertically.

#### Setting up with code
Firstly, we performed jet stream initialisation with the AStar Planner Class by modifying it to recognize and use the jet stream parameters; i.e coordinates of the minus cost area, defining the direction of the jet stream and maximum cost reduction.

Then, we established a section that systematically tests every possible vertical placement for the jet stream to find the best placement via the optimization loop. This was done by iterating every possible vertical start position for the jet stream, then redefining the jet stream area for each iteration to satisfy the lateral span. A new AStar planner is initialized in every loop iteration with the current stream coordinates and then the 'if current_time<best_time:' function compares the resulting path cost from the current jet stream placement to the running minimum.

The cost calculation logic was then performed via the planning method; whereby the 5% cost reduction was applied to the path search during the AStar algorithm. The code checks if the current node is inside the jet stream area while the discount code reduces the node cost by a maximum of 5% as it is set to 0.05.

Finally, after the loop determines the best time (The lowest cost found), the function shown in the image below takes the optimal time and uses the pandas Dataframe to perform the financial analysis based on scenario 1. 


#### Optimal Placement
#### Results

## 5. Task 3
<a href="task3.py"><strong>Task 3 Code</strong></a>
#### Introduction
#### Scenarios
#### Calculation

## 6. Additional Task 1
<a href="taska1.py"><strong>Task A1 Code</strong></a>
#### Modified Code
#### Results

## 7. Additional Task 2
<a href="taska2.py"><strong>Task A2 Code</strong></a>
#### Description
#### Results with modified code

## 8. Additional Task 3
<a href="task a3/A star task a3.py"><strong>Task A3 Astar Code</strong></a>
<a href="task a3/Dijkstra task a3.py"><strong>Task A3 Dijkstra Code</strong></a>
<a href="task a3/RRT task a3.py"><strong>Task A3 RRT Code</strong></a>
#### Description of A*, Dijkstra and RRT
#### Theories
#### Performance
#### Limitations
#### Summarize

<!-- Group Reflections -->
## 9. Group Reflections
Winkelhock, Yau Yue Hong Winkelhock (25034703D)

Rodgers, Rodgers Mawalla Maighacho (25094994D)

August, Wang Tiancheng (25095793D)

Manny, So Yee Man (25079801D)

Talia, Cheung Yung Ting Talia (25129573D)

Sharon, Chan Chin Ying (25084065D)

Sylvia, Lau Tsz Wing (25068927D)

<!-- Presentation Files -->
## 10. Presentation Files

<!-- Report -->
## 11. Report
