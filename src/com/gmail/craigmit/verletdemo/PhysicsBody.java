package com.gmail.craigmit.verletdemo;

import java.awt.geom.Point2D;

/**
 * <p>Original C++ code written by Benedikt Bitterli Copyright (c) 2009
 * <p>Original C++ code and tutorial available at {@link http://www.gamedev.net/reference/programming/features/verletPhys/default.asp}.
 * <p>Conversion from C++ to Java done by Craig Mitchell Copyright (c) 2010.
 * <p>
 * <pre>
 * The code is released under the ZLib/LibPNG license.
 * It basically means that you can treat the source in any way you like (including commercial applications),
 * but you may not claim that you wrote it.
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 *     1. The origin of this software must not be misrepresented; you must not
 *        claim that you wrote the original software. If you use this software
 *        in a product, an acknowledgment in the product documentation would be
 *        appreciated but is not required.
 *        
 *     2. Altered source versions must be plainly marked as such, and must not be
 *        misrepresented as being the original software.
 *        
 *     3. This notice may not be removed or altered from any source distribution.
 * </pre>
 * 
 * @author Craig Mitchell
 * @since 21/01/2010
 */
public class PhysicsBody {
	protected Point2D.Float center = new Point2D.Float(0, 0); //Center of mass
	protected float minX, minY, maxX, maxY; //Min/max coordinates of the bounding box
	protected int vertexCount;
	protected int edgeCount;
	protected int hasConstraint;
	protected float mass;
	protected Vertex[] vertices = new Vertex[ Physics.MAX_BODY_VERTICES ];
	protected Edge[]   edges    = new Edge[ Physics.MAX_BODY_EDGES    ];

	public PhysicsBody(Physics parent, float f) {
		vertexCount = edgeCount = hasConstraint = 0;
		mass = f;
		parent.addBody( this ); //Add body to the physics simulator
	}

	public void addEdge(Edge E) {
		edges[ edgeCount++ ] = E;
	}
	
	public void addVertex(Vertex V ) {
		vertices[ vertexCount++ ] = V;
	}

	public MinMax ProjectToAxis(Point2D.Float Axis) {
		float dotP = Axis.x * vertices[ 0 ].position.x + Axis.y * vertices[ 0 ].position.y;
		MinMax data = new MinMax();
		data.min = dotP;
		data.max = dotP; //Set the minimum and maximum values to the projection of the first vertex

		for (int i = 0; i < vertexCount; i++ ) {
			dotP = Axis.x * vertices[ i ].position.x + Axis.y * vertices[ i ].position.y; //Project the rest of the vertices onto the axis and extend the interval to the left/right if necessary
			data.min = Math.min( dotP, data.min );
			data.max = Math.max( dotP, data.max );
		}
		
		return data;
	}

	/**
	 * Calculates the center of mass
	 */
	public void calculateCenter() {
		center.x = 0;
		center.y = 0;

		minX = 10000.0f;
		minY = 10000.0f;
		maxX = -10000.0f;
		maxY = -10000.0f;

		for (int i = 0; i < vertexCount; i++) {
			center.x += vertices[ i ].position.x;
			center.y += vertices[ i ].position.y;

			minX = Math.min( minX, vertices[ i ].position.x );
			minY = Math.min( minY, vertices[ i ].position.y );
			maxX = Math.max( maxX, vertices[ i ].position.x );
			maxY = Math.max( maxX, vertices[ i ].position.y );
		}

		center.x /= vertexCount;
		center.y /= vertexCount;
	}

	/**
	 * Helper function to create a box primitive.
	 * @param x
	 * @param y
	 * @param width
	 * @param height
	 */
	public void createBox(Physics world, int x, int y, int width, int height ) {
		Vertex V1 = new Vertex( world, this, x        , y          );
		Vertex V2 = new Vertex( world, this, x + width, y          );
		Vertex V3 = new Vertex( world, this, x + width, y + height );
		Vertex V4 = new Vertex( world, this, x        , y + height );

		new Edge( world, this, V1, V2, true );
		new Edge( world, this, V2, V3, true );
		new Edge( world, this, V3, V4, true );
		new Edge( world, this, V4, V1, true );

		new Edge( world, this, V1, V3, false );
		new Edge( world, this, V2, V4, false );
	}
	
	public void addConstraints() {
		hasConstraint = 1;
	}
	public void applyConstraints() {
		float tempFactor;
		float ax_bx,ay_by,cx_bx,cy_by,ax_dx,ay_dy,ex_dx,ey_dy,cx_dx,cy_dy;
		if (hasConstraint == 1) {
			ax_bx = vertices[0].oldPosition.x - vertices[1].oldPosition.x;
			ay_by = vertices[0].oldPosition.y - vertices[1].oldPosition.y;
			cx_bx = vertices[0].position.x    - vertices[1].oldPosition.x;
			cy_by = vertices[0].position.y    - vertices[1].oldPosition.y;
			
			tempFactor = (ax_bx*cx_bx + ay_by*cy_by)/(ax_bx*ax_bx+ay_by*ay_by);
			vertices[0].position.x = vertices[1].oldPosition.x + ax_bx*tempFactor;
			vertices[0].position.y = vertices[1].oldPosition.y + ay_by*tempFactor;

			ax_bx = vertices[1].oldPosition.x - vertices[2].oldPosition.x;
			ay_by = vertices[1].oldPosition.y - vertices[2].oldPosition.y;
			cx_bx = vertices[1].position.x    - vertices[2].oldPosition.x;
			cy_by = vertices[1].position.y    - vertices[2].oldPosition.y;
			
			tempFactor = (ax_bx*cx_bx + ay_by*cy_by)/(ax_bx*ax_bx+ay_by*ay_by);
			vertices[1].position.x = vertices[2].oldPosition.x + ax_bx*tempFactor;
			vertices[1].position.y = vertices[2].oldPosition.y + ay_by*tempFactor;

			ax_bx = vertices[2].oldPosition.x - vertices[3].oldPosition.x;
			ay_by = vertices[2].oldPosition.y - vertices[3].oldPosition.y;
			cx_dx = vertices[2].position.x    - vertices[3].position.x;
			cy_dy = vertices[2].position.y    - vertices[3].position.y;
			ax_dx = vertices[2].oldPosition.x - vertices[3].position.x;
			ay_dy = vertices[2].oldPosition.y - vertices[3].position.y;
			ex_dx = ax_dx-0.4f*ax_bx;
			ey_dy = ay_dy-0.4f*ay_by;

			tempFactor = (ex_dx*cx_dx + ey_dy*cy_dy)/(ex_dx*ex_dx+ey_dy*ey_dy);
			vertices[2].position.x = vertices[3].position.x + ex_dx*tempFactor;
			vertices[2].position.y = vertices[3].position.y + ey_dy*tempFactor;
		}
	}
};
