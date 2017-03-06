/**
 *  https://sites.google.com/site/sailboatinstruments1/step-1
 */

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import java.util.Vector;


/**
 *
 */
public class CompassCalibration {
	
	//private Vector<TrippleDataInt> dataVector = new Vector<TrippleDataInt>();
	private Matrix b;
	private Matrix a_1;
	
	/*public void addData(TrippleDataInt data) {
		dataVector.add(data);
	}*/
	
	public void calculate(List<double[]> rawData) {
		Matrix d = new Matrix(10, rawData.size());
		
		double x; 
		double y;
		double z;
		for (int i=0; i< rawData.size(); i++) {
			x = rawData.get(i)[0];
			y = rawData.get(i)[1];
			z = rawData.get(i)[2];
			
			d.set(0, i, x*x);
			d.set(1, i, y*y);
			d.set(2, i, z*z);
			d.set(3, i, 2.0*y*z);
			d.set(4, i, 2.0*x*z);
			d.set(5, i, 2.0*x*y);
			d.set(6, i, 2.0*x);
			d.set(7, i, 2.0*y);
			d.set(8, i, 2.0*z);
			d.set(9, i, 1.0);
		}
		
		// S = D * Dt
		Matrix s = d.times(d.transpose());
		
		
		Matrix s11 = s.getMatrix(0, 5, 0, 5);
		Matrix s12 = s.getMatrix(0, 5, 6, 9);
		Matrix s22 = s.getMatrix(6, 9, 6, 9);
		
		Matrix s22Inv = s22.inverse();
		
		// Calculate SS = S11 - S12 * S22-1 * S12_T
		
		Matrix s22a = s22Inv.times(s12.transpose());	// First calculate S22a = S22-1  * S12T      ( 4x6 = 4x4 * 4x6)
		Matrix ss =  s11.minus(s12.times(s22a));
		
		// constraint Matrix c
		
		Matrix c = new Matrix(6,6, 0.0);
		c.set(0, 0, -1.0); c.set(0, 1,  1.0); c.set(0, 2,  1.0); 
		c.set(1, 0,  1.0); c.set(1, 1, -1.0); c.set(1, 2,  1.0);  
		c.set(2, 0,  1.0); c.set(2, 1,  1.0); c.set(2, 2, -1.0);  
		c.set(3, 3, -4.0);  
		c.set(4, 4, -4.0);  
		                                                                                               c.set(5, 5, -4.0);

		c = c.inverse();
		
		// Calculate E = C * SS      ( 6x6 = 6x6 * 6x6) 
		
		Matrix e = c.times(ss);
		
		// Calculate eigenvalues wr(6x1) and eigenvectors vr(6x6) of matrix E
		
		EigenvalueDecomposition evd = e.eig();
		
		double[] wr = evd.getRealEigenvalues();
		Matrix vr = evd.getV();
		
		// Find the zero-based position of the only positive eigenvalue. 
		// The associated eigenvector will be in the corresponding column of matrix vr(6x6). 
		 
		int index = 0;
		double maxval = wr[0];
		for (int i = 1; i < 6; i++) {
			if(wr[i] > maxval) {
				maxval = wr[i];
				index = i;
			}
		}		
		
		// Extract the associated eigenvector v1
		 
		Matrix v1 = new Matrix(6,1);

		v1.set(0,0, vr.get(0, index)); 
		v1.set(1,0, vr.get(1, index)); 
		v1.set(2,0, vr.get(2, index)); 
		v1.set(3,0, vr.get(3, index)); 
		v1.set(4,0, vr.get(4, index)); 
		v1.set(5,0, vr.get(5, index)); 

		// Check sign of eigenvector v1
		 
		if (v1.get(0, 0) < 0.0)
			v1 = v1.times(-1.0);
		
		// Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
		
		Matrix v2 = s22a.times(v1);
		
		// Setup vector v
		 
		double[] v = new double[10];
		v[0] = v1.get(0, 0);
		v[1] = v1.get(1, 0);
		v[2] = v1.get(2, 0);
		v[3] = v1.get(3, 0);
		v[4] = v1.get(4, 0);
		v[5] = v1.get(5, 0);
		v[6] = -v2.get(0, 0);
		v[7] = -v2.get(1, 0);
		v[8] = -v2.get(2, 0);
		v[9] = -v2.get(3, 0);		
		
		/***************************
		At this point, we have found the general equation of the fitted ellipsoid:
		A*x^2 + B*y^2 + C*z^2 + D*x*y + E*x*z * F*y*z + 2*G*x + 2*H*y + 2*I*z + J = 0
		
		where:
		    A = v[0]        - term in x2
		    B = v[1]        - term in y2
		    C = v[2]        - term in z2
		    D = v[5]        - term in xy  
		    E = v[4]        - term in xz
		    F = v[3]        - term in yz
		    G = v[6]        - term in x
		    H = v[7]        - term in y
		    I = v[8]        - term in z
		    J = v[9]        - constant term
		    
		    
		     A D E       G
		 Q = D B F   U = H
		     E F C       I 
		     
		     
		then the center of the ellipsoid can be calculated as the vector    
		
		B = - Q-1 * U
		**********************/
		
		Matrix q = new Matrix(3,3);
		q.set(0, 0, v[0]); q.set(0, 1, v[5]); q.set(0, 2, v[4]); 
		q.set(1, 0, v[5]); q.set(1, 1, v[1]); q.set(1, 2, v[3]); 
		q.set(2, 0, v[4]); q.set(2, 1, v[3]); q.set(2, 2, v[2]); 
		
		Matrix u = new Matrix(3,1);
		u.set(0, 0, v[6]);
		u.set(1, 0, v[7]);
		u.set(2, 0, v[8]);
		
		b = q.inverse().times(u).times(-1.0);
		//NumberFormat nf = new DecimalFormat("##0.000000");
		//b.transpose().print(nf, 10);
		
		// A-1 = H / sqrt(B_T*Q*B - J * Q^0.5
		
		// First calculate B_T * Q * B   ( 1x1 = 1x3 * 3x3 * 3x1)
		
		double value = b.transpose().times(q.times(b)).get(0,0);
		value = Math.sqrt(value - v[9]);
		value = 1000.0 / value;  		
		//value = 1.0 / value;
		
		Matrix t = q.eig().getV();
		d = q.eig().getD();
		d.set(0,0, Math.sqrt(d.get(0,0)));
		d.set(1,1, Math.sqrt(d.get(1,1)));
		d.set(2,2, Math.sqrt(d.get(2,2)));
		
		a_1 = t.times(d.times(t.inverse()));
		a_1 = a_1.times(value);
		
		//a_1.print(nf, 10);
	}

	public Matrix getMatrixB() {
		return b;
	}

	public Matrix getMatrixA_1() {
		return a_1;
	}
		
}