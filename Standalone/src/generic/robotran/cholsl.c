//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

void cholsl(double **a, int n, double p[], double b[], double x[])
{
	int i,k;
	double sum;

	for (i=1;i<=n;i++) {
		for (sum=b[i],k=i-1;k>=1;k--) sum -= a[i][k]*x[k];
		x[i]=sum/p[i];
	}
	for (i=n;i>=1;i--) {
		for (sum=x[i],k=i+1;k<=n;k++) sum -= a[k][i]*x[k];
		x[i]=sum/p[i];
	}
}
