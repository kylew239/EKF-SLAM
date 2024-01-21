# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

            1: A standalone function
               pros: Makes it easier to test and be reused
               cons: The function is not encapsulated by any class or struct, so the overall code becomes less organized


            2: Operator overload for Vector2D
               pros: Makes it easier to use and more organized
               cons: Doesn't make as much sense to use operator overload

            3: A member function of Vector2D
               pros: Gives you control over Vector2D objects and behaviors
               cons: Need to turn Vector2D into a class

            I chose to use make it a standalone function because it is easier to implement and test, and I don't really need the benefits of the other methods.

2. What is the difference between a class and a struct in C++?

        A class has both data and functions, while a struct only has data. In a class, the data is usually only accessible by the functions of the class. Data within a struct however, is accesible to any function/user.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

        Vector is a struct because it just has data (x and y). Transform2D is a class because it has data and methods. The core guidelines say to use structs if the data members can vary independently, like in the Vectors2D (C.2). The data members within the Transform2D however, are dependent. 

        The guidelines also say to use class over struct if there are non-public members, which the Transform2D has (C.8)


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
        
        The C++ core guidelines say to declare single-argument constructors as explicit. This is done to help prevent unintended conversions. (C.46)


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?

    Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

        Transform2D::inv() returns a new Transform2D object, it does not modify the provided argument. The C++ Core guidelines say to make members const by default (Con.2)

        Transform2D::opeartor*=() is not a const because it does modify the original argument.
        
