# -*- coding: utf-8 -*-

from copy import deepcopy
import numpy

class Structure:
    # This code implement the articel :
    #   "ModQuad: The Flying Modular Structure that Self-Assembles in Midair", 
    #   David Saldaña, Bruno Gabrich, Guanrui Li, Mark Yim, and Vijay Kumar.
    #   ICRA'2018 - IEEE International Conference on Robotics and Automation, At Brisbane, Australia
    #
    # In the present code, the article structure have been adapted to the px4 structures.
    # All the details of that adapation are explains in the file 
    # "adaptation_of_flying_modulue_structure_to_px4.org" taht is present with that source
    # code.

    def __init__( self, rotor_offset, module_size, module_mass, px4_quad_rotors, rotor_permutation, module_discrete_coordinates_2D ):
        """
                 
           +-----------------+
           |                 |
           |                 |
           |   X ------- X   |
           |   |         |   |
           |   |         |   | 
           |   |         |   |
           |   X ------- X   |
           |        < ro >   |
           |                 |
           +-----------------+
           <- - -  ms - - - ->    
        #
           legend :
               ro : rotor_offset
               X  : Rotors position
               ms  : module_size
        
        
        rotor_offset : x-distance between drone center and rotor
        
        module_size : the modula have to be a square. It is the size of the suqare
        module_mass : the mass of one module
        px4_quad_rotors : the matrix that convert a control to a force for one module
        rotor_permutation : the position of the rotors in px4
             rotor_permutation = [p1, p2, p3, p4]
             p4 --- p1
             |      |
             |      |
             p3 --- p2 
           For example for qud_x.toml, it is [1, 4, 2, 3]
        module_discrete_coordinates_2D : the position of the modules expressed in the px4 frame.
        
        The px4 frame are oriented such that : 
           - x axis is oriented towards the front 
           - y axis is oriented to the right
        
        
        Example :
        
            >>> S = Structure(
            ...     rotor_offset = 0.707107,
            ...     module_size = 2.0,
            ...     module_mass = 1.0,
            ...     px4_quad_rotors = [
            ...         [ -0.707107,  0.707107,  1.000000,  1.000000 ],
            ...         [  0.707107, -0.707107,  1.000000,  1.000000 ],
            ...         [  0.707107,  0.707107, -1.000000,  1.000000 ],
            ...         [ -0.707107, -0.707107, -1.000000,  1.000000 ],
            ...     ],
            ...     rotor_permutation = [1, 4, 2, 3],
            ...     module_discrete_coordinates_2D = [] 
            ...     #module_discrete_coordinates_2D = [[0, 0], [0, 1],[1,1], [0,-1]] 
            ... )
            >>> S.add_module([0,0])
            >>> S.get_rotor(0,0)
            matrix([[-0.707107,  0.707107,  1.      ,  1.      ]])
            >>> S.get_rotor(0,1)
            matrix([[ 0.707107, -0.707107,  1.      ,  1.      ]])
            >>> S.get_rotor(0,2)
            matrix([[ 0.707107,  0.707107, -1.      ,  1.      ]])
            >>> S.get_rotor(0,3)
            matrix([[-0.707107, -0.707107, -1.      ,  1.      ]])
            >>> print(S.article_to_px4_force)
            >>> S.add_module([1,0])
            >>> for i in range( S.nb_of_modules() ):
            ...     print( "Rotors of drone " + str(i) + ":"  )
            ...     for j in range(4):
            ...         print( S.get_rotor(i,j) )
            Rotors of drone 0:
            [[-0.707107 -0.707107  1.        0.5     ]]
            [[ 0.707107 -0.707107  1.        0.5     ]]
            [[ 0.707107 -0.707107 -1.        0.5     ]]
            [[-0.707107 -0.707107 -1.        0.5     ]]
            Rotors of drone 1:
            [[-0.707107  0.707107  1.        0.5     ]]
            [[ 0.707107  0.707107  1.        0.5     ]]
            [[ 0.707107  0.707107 -1.        0.5     ]]
            [[-0.707107  0.707107 -1.        0.5     ]]

            >>> S.add_module([1,1])
            >>> for i in range( S.nb_of_modules() ):
            ...     print( "Rotors of drone " + str(i) + ":"  )
            ...     for j in range(4):
            ...         print( S.get_rotor(i,j) )
            Rotors of drone 0:
            [[-0.707107   -0.707107    1.          0.33333333]]
            [[ 0.707107   -0.707107    1.          0.33333333]]
            [[ 0.707107   -0.707107   -1.          0.33333333]]
            [[-0.707107   -0.707107   -1.          0.33333333]]
            Rotors of drone 1:
            [[-0.707107    0.707107    1.          0.33333333]]
            [[ 0.707107   -0.707107    1.          0.33333333]]
            [[ 0.707107    0.707107   -1.          0.33333333]]
            [[-0.707107   -0.707107   -1.          0.33333333]]
            Rotors of drone 2:
            [[-0.707107    0.707107    1.          0.33333333]]
            [[-0.707107   -0.707107    1.          0.33333333]]
            [[-0.707107    0.707107   -1.          0.33333333]]
            [[-0.707107   -0.707107   -1.          0.33333333]]
        """
        def kronecker(i, j):
            if i==j :
                return 1.0
            else:
                return 0.0
        self.px4_to_article_force = numpy.matrix(
            [
                [kronecker(i-1,j) for j in range(4)]
                for i in rotor_permutation
            ]
        )
        self.article_to_px4_force = numpy.matrix(
            [
                [kronecker(i,j-1) for j in rotor_permutation]
                for i in range(4)
            ]
        )
        self.px4_quad_rotors = numpy.matrix( px4_quad_rotors )
        self.rotor_offset = rotor_offset
        self.module_size = float( module_size )
        self.module_mass = float( module_mass )
        self.module_discrete_coordinates_2D = module_discrete_coordinates_2D
        self.update()

    def compute_P_matrix(self, position ):
        def sign(x):
            assert( x != 0.0 )
            return x/abs(x)
        return numpy.matrix( [
            [1.0, sign( position[1]-self.d ), sign( position[0]+self.d ),  1],
            [1.0, sign( position[1]-self.d ), sign( position[0]-self.d ), -1],
            [1.0, sign( position[1]+self.d ), sign( position[0]-self.d ),  1],
            [1.0, sign( position[1]+self.d ), sign( position[0]+self.d ), -1],
        ] )

    def update(self):
        self.n = len( self.module_discrete_coordinates_2D )
        
        zero = numpy.matrix( [[0.0], [0.0]] )
        e0 = numpy.matrix( [[1.0], [0.0]] )
        e1 = numpy.matrix( [[0.0], [1.0]] )

        self.d = self.rotor_offset

        self.P_for_1_module = self.compute_P_matrix( [0.0, 0.0] )
        self.inverse_of_P_for_1_module = numpy.linalg.inv( self.P_for_1_module )

        if self.n == 0:
            self.modules_in_absolute_position_2D = []
            self.structure_mass = 0.0
            self.center_of_mass_2D = zero
            self.modules = []
            self.H_n = numpy.diag( [1.0, 1.0, 1.0, 1.0] )
            self.Pi = []
            self.all_px4_rotors = []
            return 

        zero = numpy.matrix( [[0.0], [0.0]] )
        
        # We need to convert module poisiion in px4 frame into module position
        # in the frame of the article 
        #
        # so, e1 = [0,-1]
        e0 = numpy.matrix( [[1.0], [0.0]] )
        e1 = numpy.matrix( [[0.0], [-1.0]] )
        self.modules_in_absolute_position_2D = map(
            lambda position : ( e0*position[0] + e1*position[1] )*self.module_size,
            self.module_discrete_coordinates_2D
        )

        self.structure_mass = self.n * self.module_mass
        self.center_of_mass_2D = zero
        for position in self.modules_in_absolute_position_2D :
            self.center_of_mass_2D += ( position * self.module_mass )
        self.center_of_mass_2D /= self.structure_mass

        self.modules = map(
            lambda position : position - self.center_of_mass_2D,
            self.modules_in_absolute_position_2D
        )

        self.H_n = numpy.diag( [1.0/self.n, 1.0, 1.0, 1.0] )

        self.Pi = [
            self.compute_P_matrix( position )
            for position in self.modules
        ]

        self.all_px4_rotors = [
            self.article_to_px4_force * self.Pi[i] * self.H_n * self.inverse_of_P_for_1_module * self.px4_to_article_force * self.px4_quad_rotors
            for i in range( self.n )
        ]

    def get_rotor( self, id_module, id_rotor ):
        return self.all_px4_rotors[id_module][id_rotor]

    def nb_of_modules(self):
        return self.n

    def add_module( self, module_coordinate_2D ):
        self.module_discrete_coordinates_2D.append( module_coordinate_2D )
        self.update()



if __name__ == '__main__':
    import doctest
    doctest.testmod()
