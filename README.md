# TSFS12: Autonomous Vehicles – Planning, Control, and Learning Systems

This repository includes lecture notes, material for hand-in exercises, links to reading material, and other course material. Information here will be added during the course and you can always get the latest version of course material here. 

Note: (2020) by the lecture indicates that the material is from the 2020 edition and may be updated during the course. Removal of (2020) indicates that the material is for the 2021 edition.

## Introduction to computer tools in Python [[slides](Lecture_notes/computer_tools_introduction.pdf)]
### Video

1. [_Introduction to (some) computer tools for Python in TSFS12_](https://web.microsoftstream.com/video/a32e5976-f7e3-4236-abdb-047f250e19e8) [49:44]


## Lecture 1: Introduction [[slides](Lecture_notes/lecture_01_introduction.pdf)]

### Video

1. [_Introduction to autonomous systems_](https://web.microsoftstream.com/video/71e9b6fc-52b5-4dfb-8b9b-b2694fd9e81e) [12:21]
2. [_Autonomous systems - a broader context_](https://web.microsoftstream.com/video/aa413c92-1898-49d8-829e-435e1d9729ab) [16:03]
3. [_Enabling technologies_](https://web.microsoftstream.com/video/352f3cfe-93dd-4081-9c63-8fe8b80e9dd9) [11:17]
4. [_Autonomous vehicles and summary_](https://web.microsoftstream.com/video/0d748f3b-5df5-497e-bd05-11d44075268a) [18:34]

### Reading material
Recommended reading material for the introductory lecture are
* Introduction to SLAM, Part E Chapter 46.1 in Siciliano, B., and Oussama K., eds. “[_Springer handbook of robotics_](https://login.e.bibl.liu.se/login?url=https://search.ebscohost.com/login.aspx?authtype=guest&custid=s3912378&groupid=main&direct=true&db=cat00115a&AN=lkp.941644&profile=eds2&lang=sv)”. 
* "[_National Highway and Traffic Safety Administration ODI on Tesla_](https://static.nhtsa.gov/odi/inv/2016/INCLA-PE16007-7876.PDF)". Interesting text but there are parts that have met critique, see, e.g., [_NHTSA's Flawed Autopilot Safety Study Unmasked_](https://www.thedrive.com/tech/26455/nhtsas-flawed-autopilot-safety-study-unmasked) (Note: Opinion piece written by automotive industry analyst).
* A. Pernestål et al. "[_Effects of driverless vehicles-Comparing simulations to get a broader picture_](https://d1rkab7tlqy5f1.cloudfront.net/TBM/Over%20faculteit/Afdelingen/Engineering%20Systems%20and%20Services/EJTIR/Back%20issues/19.1/362018%20Pernestal.pdf)" European Journal of Transport & Infrastructure Research 19.1 (2019).
* Michon, John A. "[_A critical view of driver behavior models: what do we know, what should we do?_](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.473.3166&rep=rep1&type=pdf)." Human behavior and traffic safety. Springer, Boston, MA, 1985. 485-524.
* SAE, "[_Taxonomy and Definitions for Terms Related to Driving Automation Systems for On-Road Motor Vehicles_](https://login.e.bibl.liu.se/login?url=https://saemobilus.sae.org/content/J3016_201806/)", Standard J3016.
* Recommends an interesting discussion on [_The Artificial Intelligence podcast_](https://lexfridman.com/ai/) with Sertac Karaman https://lexfridman.com/sertac-karaman/ on autonomous driving and flying. Sertac Karaman is also author of course litterature in lecture 4.

## Lecture 2: Discrete motion planning [[slides](Lecture_notes/lecture_02_discrete_motion_planning.pdf)]

### Video

1. [_Introduction to graph search_](https://web.microsoftstream.com/video/05245e59-5f68-4109-8a27-5b0589a74956) [16:32]
2. [_Dijkstra's algorithm_](https://web.microsoftstream.com/video/f8b4d2d4-7f2e-44b2-8817-abb93b7993ff) [14:04]
3. [_A* algorithm_](https://web.microsoftstream.com/video/acedd6e1-d21c-4413-adb3-d39a98923910) [17:35]
4. [_Anytime planning and conclusions_](https://web.microsoftstream.com/video/347259f7-ab50-4994-82a5-dc5e67a51100) [08:59]

### Reading material
Main text is Chapter 2 (mainly sections 2.1-2.3) in "[_Planning Algorithms_](http://planning.cs.uiuc.edu)", S. Lavalle. 

For the interested reader, suggested material to dig deeper is
* Likhachev et al. "[_ARA*: Anytime A* with provable bounds on sub-optimality_](http://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf)", Advances in neural information processing systems. 2004.
* Introductory section in Bertsekas, D.  "[_Reinforcement learning and optimal control_](https://web.mit.edu/dimitrib/www/RLbook.html)", 2019, Athena, is used in the exercise for higher grade in hand-in 1. A PDF with the introductory text can be found on the authors book-page at
https://web.mit.edu/dimitrib/www/RL_1-SHORT-INTERNET-POSTED.pdf
* Chen, Mo, et al. "[_Priority queues and Dijkstra's algorithm_](https://www.researchgate.net/profile/Vijaya_Ramachandran/publication/250152101_Priority_Queues_and_Dijkstra's_Algorithm/links/54170a0a0cf2218008bec462.pdf)". Computer Science Department, University of Texas at Austin, 2007. 

## Lecture 3: Modelling of ground vehicles [[slides](Lecture_notes/lecture_03_ground_vehicles.pdf)]

### Video
1. [_Mathematical modelling_](https://web.microsoftstream.com/video/832e4af2-a25c-4186-9cef-a5b44e4e6b96) [6:33]
2. [_Modelling of ground vehicles, part I_](https://web.microsoftstream.com/video/7673fea9-9e4a-4f8b-b96b-f60626061fab) [12:16]
3. [_Modelling of ground vehicles, part II_](https://web.microsoftstream.com/video/713d7a05-04c4-47ec-a17c-4fbfb1dc1e4b) [14:08]

### Reading material
* Sections 13.1 and 15.3-15-4 in LaValle, S. M.: [_Planning Algorithms_](http://planning.cs.uiuc.edu). Cambridge University Press, 2006.

## Lecture 4: Motion planning with differential constraints [[slides](Lecture_notes/lecture_04_motion_planning_with_diff_constraints.pdf)]

### Video

1. [_Lecture 4a: Introduction to motion planning with differential constraints_](https://web.microsoftstream.com/video/609ec774-5336-40c3-9b7e-b1895b0adbd6) [19:18]
2. [_Lecture 4b: Motion planning using RRTs_](https://web.microsoftstream.com/video/9bf1ca4c-cc4a-4dbc-a6c7-283fbb44350a) [24:55]
3. [_Lecture 4c: Motion planning using motion primitives and state lattices_](https://web.microsoftstream.com/video/7be3ea03-d1e6-4c69-b2c8-afad2da3c5c8) [20:50]
4. [_Lecture 4d: Path-constrained trajectory planning_](https://web.microsoftstream.com/video/1e6dcf03-b742-4b55-82b1-4b5428e0c30a) [15:10]

### Reading material
* Sections 5.1-5.5 and 14.1-14.4 in LaValle, S. M: [_Planning Algorithms_](http://planning.cs.uiuc.edu). Cambridge University Press, 2006.
* Sections 3.3.3 and 5 in Karaman, S., & E. Frazzoli: ”[_Sampling-based algorithms for optimal motion planning_](https://login.e.bibl.liu.se/login?url=https://doi.org/10.1177%2F0278364911406761)". The International Journal of Robotics Research, 30(7), 846-894, 2011.
* Section 4.5 in Bergman, K: ”[_Exploiting Direct Optimal Control for Motion Planning in Unstructured Environments_](https://doi.org/10.3384/diss.diva-174175)”, Ph.D Thesis No. 2133, Div. Automatic Control, Linköping Univ., 2021.

Additional material for further reading:
* Likhachev, M., & D. Ferguson: ”[_Planning long dynamically feasible maneuvers for autonomous vehicles_](https://login.e.bibl.liu.se/login?url=https://doi.org/10.1177%2F0278364909340445)". The International Journal of Robotics Research, 28(8), 933-945, 2009.
* Dahl, O.: ”[_Path Constrained Robot Control_](https://lucris.lub.lu.se/ws/files/4364453/8566341.pdf)”, Ph.D. Thesis, Dept. Automatic Control, Lund Univ., 1992.
* Ljungqvist, O.: ”[_Motion planning and feedback control techniques with applications to long tractor-trailer vehicles_](https://doi.org/10.3384/diss.diva-165246)”, Ph.D. Thesis, Div. Automatic Control, Linköping Univ., 2020.
* Dubins, L. E., "[_On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents_](https://doi.org/10.2307/2372560)", American Journal of Mathematics, 79(3), 497-516, 1957. 

## Lecture 5: Dynamic optimization as a tool for motion planning and control [[slides](Lecture_notes/lecture_05_dynamic_optimization_motion_planning_control.pdf)]

### Video

1. [_Lecture 5a: Introduction to dynamic optimization for motion planning and control_](https://web.microsoftstream.com/video/a2cc8445-0a2a-483b-8b8c-bc9f9af35dad) [23:44]
2. [_Lecture 5b: Challenges and solution strategies for dynamic optimization_](https://web.microsoftstream.com/video/c0ef80c2-9fdb-4420-9488-b95279c0941c) [22:22]
3. [_Lecture 5c: Solution of the resulting non-linear program_](https://web.microsoftstream.com/video/cb8c667a-8c8d-4259-a4b2-10d267edbce1) [22:23]
4. [_Lecture 5d: Case study on dynamic optimization for motion primitives_](https://web.microsoftstream.com/video/ac303d59-bec0-4e1f-ba35-f9cde068a6e7) [14:57]

### Reading material
* Limebeer, D. J., & A. V. Rao: ”[_Faster, higher, and greener: Vehicular optimal control_](https://login.e.bibl.liu.se/login?url=https://doi.org/10.1109/MCS.2014.2384951)”. IEEE Control Systems Magazine, 35(2), 36-56, 2015.

For a more mathematical treatment of the topic of numerical optimal control and further reading on the methods presented in this lecture:
* Chapter 8 in Rawlings, J. B., D. Q. Mayne, & M. Diehl: [_Model Predictive Control: Theory, Computation, and Design_](https://sites.engineering.ucsb.edu/~jbraw/mpc/). Nob Hill Publishing, 2017
* Diehl, M.: [_Numerical Optimal Control_](https://www.fs.isy.liu.se/Edu/Courses/NumericalOptimalControl/Diehl_NumOptiCon.pdf), Optec, K.U. Leuven, Belgium, 2011.

Additional material for further reading:
* Bergman, K: ”[_Exploiting Direct Optimal Control for Motion Planning in Unstructured Environments_](https://doi.org/10.3384/diss.diva-174175)”, Ph.D Thesis No. 2133, Div. Automatic Control, Linköping Univ., 2021.
* Berntorp, K., Olofsson, B., Lundahl, K., & Nielsen, L., "[_Models and methodology for optimal trajectory generation in safety-critical road-vehicle manoeuvres_](https://doi.org/10.1080/00423114.2014.939094)". Vehicle System Dynamics, 52(10), 1304-1332, 2014.
* Fors, V., "[_Autonomous Vehicle Maneuvering at the Limit of Friction_](https://doi.org/10.3384/diss.diva-170606)", Ph.D. Thesis No. 2102, Division of Vehicular Systems, Linköping University, 2020.
* Nocedal, J., & S. Wright: [_Numerical Optimization_](https://link.springer.com/book/10.1007%2F978-0-387-40065-5). Springer, 2006.

## Lecture 6: Control of autonomous vehicles I: Ground vehicles [[slides](Lecture_notes/lecture_06_ground_vehicle_motion_control.pdf)]

### Video

1. [_Introduction to paths and trajectories_](https://web.microsoftstream.com/video/901528e0-8d80-44ae-8ead-be358ee9c409) [19:35]
2.	[_Pure pursuit path controller_](https://web.microsoftstream.com/video/309de208-0cc1-4bec-b347-2ef828456946) [08:47]
3.	[_State-feedback path controller_](https://web.microsoftstream.com/video/ee95a473-bc53-4e81-b232-17ceb88abd7e) [16:32]
4.	[_Tuning, non-linear control, and summary_](https://web.microsoftstream.com/video/5a880726-36ab-4a41-b30e-0f040f99f416) [09:21]
5.	[_Derivation of Frenet equations_](https://web.microsoftstream.com/video/79487d17-8f66-475f-b1ae-82c2ec875664) [13:23]

### Reading material
Main texts are
* Paden, Brian, et al. "[_A survey of motion planning and control techniques for self-driving urban vehicles_](https://doi.org/10.1109/TIV.2016.2578706)". IEEE Transactions on intelligent vehicles 1.1 (2016): 33-55.

    A good but slightly advanced text and therefore many details are included in the lecture notes.

* Coulter, R. Craig. "[_Implementation of the Pure Pursuit Path Tracking Algorithm_](https://apps.dtic.mil/dtic/tr/fulltext/u2/a255524.pdf)" (1992). 


    But don’t look at figure 1, it is badly scaled which makes it difficult to understand; use figures in the lecture slides instead

For the interested reader, suggested material to dig deeper is
* Siciliano, B., and Oussama K., eds. “[_Springer handbook of robotics_](https://login.e.bibl.liu.se/login?url=https://search.ebscohost.com/login.aspx?authtype=guest&custid=s3912378&groupid=main&direct=true&db=cat00115a&AN=lkp.941644&profile=eds2&lang=sv)”. Springer, 2016. Part E, Chapters 49 (wheeled robots), 51 (underwater), and 52 (aerial) 

* Werling, M., Gröll, L., and Bretthauer, G.. "[_Invariant trajectory tracking with a full-size autonomous road vehicle_](https://doi.org/10.1109/TRO.2010.2052325)". IEEE Transactions on Robotics 26.4 (2010): 758-765. 
    An advanced text, nonlinear trajectory stabilizing controllers.

## Lecture 7 (2020): Model predictive control for autonomous vehicles [[slides](Lecture_notes/lecture_07_model_predictive_control_for_autonomous_vehicles.pdf)]
### Video

1. [_Lecture 7a: Introduction to model predictive control for autonomous vehicles_](https://web.microsoftstream.com/video/de248705-1dc8-46bf-8b61-27ddc781d504) [12:26]
2. [_Lecture 7b: Fundamentals of model predictive control_](https://web.microsoftstream.com/video/515dc9f4-977a-414a-a83a-c8d9effe4247) [20:34]
3. [_Lecture 7c: Autonomous trajectory tracking and path following using MPC_](https://web.microsoftstream.com/video/dfbd498f-8384-4b19-b11d-b38f970b9b16) [26:24]
4. [_Lecture 7d: Summary, outlook, and tools for MPC in autonomous vehicles_](https://web.microsoftstream.com/video/fb92a8d0-9503-44b6-b78d-70d75a345f2e) [09:48]

### Reading material (2020)
* Chapter 1 in Rawlings, J. B., D. Q. Mayne, & M. Diehl: [_Model Predictive Control: Theory, Computation, and Design_](https://sites.engineering.ucsb.edu/~jbraw/mpc/). Nob Hill Publishing, 2017
* Section V-C in Paden, B., et al.: "[_A survey of motion planning and control techniques for self-driving urban vehicles_](https://doi.org/10.1109/TIV.2016.2578706)". IEEE Transactions on Intelligent Vehicles, 1(1), 33-55, 2016.

Additional material for further reading:
* Berntorp, K., Quirynen, R., Uno, T., & Di Cairano, S: "[_Trajectory tracking for autonomous vehicles on varying road surfaces by friction-adaptive nonlinear model predictive control_](https://doi.org/10.1080/00423114.2019.1697456)". Vehicle System Dynamics, 58(5), 705-725, 2020.
* Diehl, M., Bock, H. G., & Schlöder, J. P: ”[_A real-time iteration scheme for nonlinear optimization in optimal feedback control_](https://doi.org/10.1137/S0363012902400713)”. SIAM Journal on Control and Optimization, 43(5), 1714-1736, 2005.


## Lecture 8: Collaborative control [[slides](Lecture_notes/lecture_08_collaborative_control.pdf)]

### Video
1. [_Introduction to Collaborative Control_](https://web.microsoftstream.com/video/4c1bf4f3-6908-4de8-92b1-9a9d140a618e) [10:45]
2. [_Position Based Collaborative Control_](https://web.microsoftstream.com/video/fb7188f4-187b-4a5a-9074-a85eafe4781f) [04:08]
3. [_Displacement Based Collaborative Control_](https://web.microsoftstream.com/video/865a6c91-cf33-415b-a4b8-01e68dfad717) [07:26]
4. [_Distance Based Collaborative Control_](https://web.microsoftstream.com/video/cd5c95fb-4b73-4e73-94a7-39e1343a8a46) [13:14]

### Reading material
Background and history can be found in "[_Springer handbook of robotics_](https://login.e.bibl.liu.se/login?url=https://search.ebscohost.com/login.aspx?authtype=guest&custid=s3912378&groupid=main&direct=true&db=cat00115a&AN=lkp.941644&profile=eds2&lang=sv)”. Springer, 2016. Part E, Chapter 53 (Multiple Mobile Robot Systems).

The methods described in the lecture can be found in:
* Oh, K. K., Park, M. C., & Ahn, H. S.: "[_A survey of multi-agent formation control_](https://doi.org/10.1016/j.automatica.2014.10.022)". Automatica, 53, 424-440, 2015.


## Lecture 9 (2020): Control of autonomous vehicles II [[slides](Lecture_notes/lecture_09_control_II.pdf)]

### Video
 
 1. [_Closed Loop RRT_](https://web.microsoftstream.com/video/19e72f80-3a5c-408f-87db-2b2ac5b5c171) [15:14]

### Reading material
Main text is
* Kuwata, Y., Teo, J., Fiore, G., Karaman, S., Frazzoli, E., & How, J. P.: "[_Real-Time Motion Planning With Applications to Autonomous Urban Driving_](https://doi.org/10.1109/TCST.2008.2012116)". IEEE Transactions on Control Systems Technology, 17(5), 1105-1118, 2009.


## Lecture 10-11 (2020): Learning for autonomous vehicles [[slides-intro](Lecture_notes/lecture_10_learning_intro.pdf), [slides](Lecture_notes/lecture_10_11_learning_methods.pdf)]
### Video

1. [_Introduction to learning for autonomous systems_](https://web.microsoftstream.com/video/393a9866-d1b4-47df-bef4-f11608c6c43f) [27:24]
2. [_Learning using neural networks_](https://web.microsoftstream.com/video/f307716d-6191-4b76-9424-61a31e5c740f) [30:39]
3. [_Learning using Gaussian processes_](https://web.microsoftstream.com/video/2fc7addd-82d3-4f7d-bdaf-17b281654ff4) [17:01]
4. [_Introduction to reinforcement learning_](https://web.microsoftstream.com/video/ddc4f5ae-7ccf-4d3c-951f-981c5027c134) [31:57]

### Reading material

* "[_Reflections on the Learning-to-Control Renaissance_](https://youtu.be/IEZFwh8sw8s)", Plenary Talk from the June 2020 IFAC World Congress by [Benjamin Recht](https://www2.eecs.berkeley.edu/Faculty/Homepages/brecht.html), UC Berkley, USA. (https://youtu.be/IEZFwh8sw8s)
* Sections 11.2-11.8 in Hastie, T., R. Tibshirani, J. Friedman, & J. Franklin: [_The Elements of Statistical Learning: Data Mining, Inference and Prediction_](https://web.stanford.edu/~hastie/ElemStatLearn/). 2nd Edition, Springer, 2005.
* Sections 1 and 2.1-2.3 in Rasmussen, C. E., & C. K. I. Williams: [_Gaussian Processes for Machine Learning_](http://gaussianprocess.org/gpml/chapters/). MIT Press, 2006.
* Sections 1, 4.1-4.4, and 6.1-6.5 in Sutton, R. S., & A. G. Barto: [_Reinforcement learning: An introduction_](http://incompleteideas.net/book/the-book-2nd.html). MIT Press, 2018.

Further reading on deep neural networks and Markov decision processes under uncertain state information:
* Goodfellow, I., Y. Bengio, & A. Courville: [_Deep Learning_](http://www.deeplearningbook.org). MIT Press, 2016.
* Åström, K. J.: ”[_Optimal control of Markov processes with incomplete state information_](https://doi.org/10.1016/0022-247X(65)90154-X)”, Journal of Mathematical Analysis and Applications, 10(1), 174-205, 1965.

## Lecture 11: Guest lecture

* Industrial guest lecturer: Dr. Karl Berntorp, Principal Research Scientist, Mitsubishi Electric Research Labs, Boston, MA (https://www.merl.com/people/berntorp)

### Reading material

* Berntorp, K., T. Hoang, R. Quirynen, & S. Di Cairano: "[_Control Architecture Design for Autonomous Vehicles_](https://doi.org/10.1109/CCTA.2018.8511371)". IEEE Conference on Control Technology and Applications (CCTA), Copenhagen, 404-411, 2018.
* Berntorp, K., T. Hoang, & S. Di Cairano: "[_Motion Planning of Autonomous Road Vehicles by Particle Filtering_](https://doi.org/10.1109/TIV.2019.2904394)". IEEE Transactions on Intelligent Vehicles, 4(2), 197-210, 2019.
* Danielson, C., K. Berntorp, A. Weiss, & S. Di Cairano: "[_Robust Motion Planning for Uncertain Systems With Disturbances Using the Invariant-Set Motion Planner_](https://doi.org/10.1109/TAC.2020.3008126)". IEEE Transactions on Automatic Control, 65(10), 4456-4463, 2020.
* Quirynen, R., K. Berntorp, K. Kambam, & S. Di Cairano: "[_Integrated Obstacle Detection and Avoidance in Motion Planning and Predictive Control of Autonomous Vehicles_](https://doi.org/10.23919/ACC45564.2020.9147820)". American Control Conference (ACC), Denver, CO, USA, 1203-1208, 2020.
* Berntorp, K., R. Bai, K. F. Erliksson, C. Danielson, A. Weiss, & S. Di Cairano: "[_Positive Invariant Sets for Safe Integrated Vehicle Motion Planning and Control_](https://doi.org/10.1109/TIV.2019.2955371)". IEEE Transactions on Intelligent Vehicles, 5(1), 112-126, 2020.

## Research guest lecture: Motion planning and differential flatness [[slides](Lecture_notes/research_guest_lecture.pdf)]

* Guest lecture given by Marcus Greiff, Department of Automatic Control, Lund University (http://www.control.lth.se/personnel/marcus-greiff/)

### Reading material

* Greiff, M.: ”[_Modelling and control of the Crazyflie quadrotor for aggressive and autonomous flight by optical flow driven state estimation_](http://lup.lub.lu.se/student-papers/record/8905295)”, Master’s Thesis, Department of Automatic Control, Lund University, 2017.
* Greiff, M.: ”[_A Time-Warping Transformation for Time-Optimal Movement in Differentially Flat Systems_](https://doi.org/10.23919/ACC.2018.8431230)”. Proc. American Control Conference (ACC), 6723-6730, 2018.
* Fliess, M., Lévine, J., Martin, P., & Rouchon, P.: ”[_Flatness and defect of non-linear systems: Introductory theory and examples_](https://doi.org/10.1080/00207179508921959)”. International Journal of Control, 61(6), 1327-1361, 1995.
* Richter, C., Bry, A., & Roy, N.: ”[_Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments_](https://doi.org/10.1007/978-3-319-28872-7_37)”. Proc. International Symposium of Robotics Research, 649-666, 2016.

Additional suggestions for further reading are available in the lecture slides.
