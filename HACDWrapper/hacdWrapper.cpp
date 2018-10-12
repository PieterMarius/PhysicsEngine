#include <Windows.h>
#include <string>
#include <array>
#include <iostream>
#include <hacdHACD.h>
#include <hacdMicroAllocator.h>
#include <exception>

#ifdef _WIN32
#define PATH_SEP "\\"
#else
#define PATH_SEP "/"
#endif


void CallBack(const char * msg, double progress, double concavity, size_t nVertices)
{
	std::cout << msg;
}

bool LoadOFF(
	const std::string & fileName, 
	std::vector< HACD::Vec3<HACD::Real> > & points, 
	std::vector< HACD::Vec3<long> > & triangles, 
	bool invert)
{
	FILE * fid;
	errno_t err = fopen_s(&fid, fileName.c_str(), "r");
	if (fid)
	{
		const std::string strOFF("OFF");
		char temp[1024];
		fscanf_s(fid, "%s", temp, _countof(temp));

		if (std::string(temp) != strOFF)
		{
			printf("Loading error: format not recognized \n");
			fclose(fid);

			return false;
		}
		else
		{
			int nv = 0;
			int nf = 0;
			int ne = 0;
			fscanf_s(fid, "%i", &nv);
			fscanf_s(fid, "%i", &nf);
			fscanf_s(fid, "%i", &ne);
			points.resize(nv);
			triangles.resize(nf);
			float x = 0;
			float y = 0;
			float z = 0;
			for (long p = 0; p < nv; p++)
			{
				fscanf_s(fid, "%f", &x);
				fscanf_s(fid, "%f", &y);
				fscanf_s(fid, "%f", &z);
				points[p].X() = x;
				points[p].Y() = y;
				points[p].Z() = z;
			}
			int i = 0;
			int j = 0;
			int k = 0;
			int s = 0;
			for (long t = 0; t < nf; ++t) {
				fscanf_s(fid, "%i", &s);
				if (s == 3)
				{
					fscanf_s(fid, "%i", &i);
					fscanf_s(fid, "%i", &j);
					fscanf_s(fid, "%i", &k);
					triangles[t].X() = i;
					if (invert)
					{
						triangles[t].Y() = k;
						triangles[t].Z() = j;
					}
					else
					{
						triangles[t].Y() = j;
						triangles[t].Z() = k;
					}
				}
				else			// Fix me: support only triangular meshes
				{
					for (long h = 0; h < s; ++h) fscanf_s(fid, "%i", &s);
				}
			}
			fclose(fid);
		}
	}
	else
	{
		printf("Loading error: file not found \n");
		return false;
	}
	return true;
}

bool LoadOFF(
	const std::string & fileName,
	double** points,
	int* nPoints,
	long** triangles,
	int* nTriangles,
	bool invert)
{
	FILE * fid;
	errno_t err = fopen_s(&fid, fileName.c_str(), "r");
	if (fid)
	{
		const std::string strOFF("OFF");
		char temp[1024];
		fscanf_s(fid, "%s", temp, _countof(temp));

		if (std::string(temp) != strOFF)
		{
			printf("Loading error: format not recognized \n");
			fclose(fid);

			return false;
		}
		else
		{
			int nv = 0;
			int nf = 0;
			int ne = 0;
			fscanf_s(fid, "%i", &nv);
			fscanf_s(fid, "%i", &nf);
			fscanf_s(fid, "%i", &ne);
			points = (double**)::CoTaskMemAlloc(sizeof(double*) * nv);
			triangles = (long**)::CoTaskMemAlloc(sizeof(long*) * nf);
			*nPoints = nv;
			*nTriangles = nf;
			float x = 0;
			float y = 0;
			float z = 0;
			for (long p = 0; p < nv; p++)
			{
				fscanf_s(fid, "%f", &x);
				fscanf_s(fid, "%f", &y);
				fscanf_s(fid, "%f", &z);
				points[p] = (double(*))::CoTaskMemAlloc(sizeof(double) * 3);
				points[p][0] = x;
				points[p][1] = y;
				points[p][2] = z;
			}
			int i = 0;
			int j = 0;
			int k = 0;
			int s = 0;
			for (long t = 0; t < nf; ++t) {
				fscanf_s(fid, "%i", &s);
				if (s == 3)
				{
					fscanf_s(fid, "%i", &i);
					fscanf_s(fid, "%i", &j);
					fscanf_s(fid, "%i", &k);
					triangles[i] = (long(*))::CoTaskMemAlloc(sizeof(long) * 3);
					triangles[t][0] = i;
					if (invert)
					{
						triangles[t][1] = k;
						triangles[t][2] = j;
					}
					else
					{
						triangles[t][1] = j;
						triangles[t][2] = k;
					}
				}
				else			// Fix me: support only triangular meshes
				{
					for (long h = 0; h < s; ++h) fscanf_s(fid, "%i", &s);
				}
			}
			fclose(fid);
		}
	}
	else
	{
		printf("Loading error: file not found \n");
		return false;
	}
	return true;
}


extern "C"
{
	__declspec(dllexport) void ExecuteHACD(
		const char* ifilename, 
		double**** pointsOut, 
		long**** triangleOut,
		int* nCluster,
		int** nOutPoints,
		int** nOutTriangles)
	{
		HACD::HeapManager * heapManager = HACD::createHeapManager(65536 * (1000));
		HACD::HACD * const myHACD = HACD::CreateHACD(heapManager);

		int nClusters = 1;

		std::vector< HACD::Vec3<HACD::Real> > points;
		std::vector< HACD::Vec3<long> > triangles;

		const std::string fileName(ifilename);
		std::string folder;
		int found = fileName.find_last_of(PATH_SEP);
		std::string file(fileName.substr(found + 1));

		std::cout << "sono qui" << "\t" << fileName << std::endl;

		if (found != -1)
		{
			folder = fileName.substr(0, found);
		}
		if (folder == "")
		{
			folder = ".";
		}

		bool invert = false;

		LoadOFF(fileName, points, triangles, invert);
				
		/*
		const std::string fileName("/Users/khaledmammou/Dev/HACD/data/Sketched-Brunnen.off");
		size_t nClusters = 1;
		double concavity = 100.0;
		bool invert = false;
		bool addExtraDistPoints = true;
		bool addNeighboursDistPoints = false;
		bool addFacesPoints = true;
		double ccConnectDist = 30.0;
		size_t targetNTrianglesDecimatedMesh = 1000;
		*/

		myHACD->SetPoints(&points[0]);
		myHACD->SetNPoints(points.size());
		myHACD->SetTriangles(&triangles[0]);
		myHACD->SetNTriangles(triangles.size());
		myHACD->SetCompacityWeight(0.0001);
		myHACD->SetVolumeWeight(0.0);

		// if two connected components are seperated by distance < ccConnectDist
		// then create a virtual edge between them so the can be merged during 
		// the simplification process
		myHACD->SetConnectDist(30.0);

		myHACD->SetNClusters(nClusters);				// minimum number of clusters
		myHACD->SetNVerticesPerCH(100);                 // max of 100 vertices per convex-hull
		myHACD->SetConcavity(100.0);                    // maximum concavity
		myHACD->SetSmallClusterThreshold(0.25);			// threshold to detect small clusters
		myHACD->SetNTargetTrianglesDecimatedMesh(1000); // # triangles in the decimated mesh
		myHACD->SetCallBack(&CallBack);
		myHACD->SetAddExtraDistPoints(true);
		myHACD->SetAddFacesPoints(true);

		myHACD->Compute();

		std::cout << "Compute" << std::endl;

		*nCluster = nClusters;
		*nOutPoints = (int*)::CoTaskMemAlloc(sizeof(int) * nClusters);
		*nOutTriangles = (int*)::CoTaskMemAlloc(sizeof(int) * nClusters);
		*pointsOut= (double***)::CoTaskMemAlloc(sizeof(double**) * nClusters);
		*triangleOut = (long***)::CoTaskMemAlloc(sizeof(long**) * nClusters);

		//TODO modificare nClusters
		for (size_t c = 0; c < nClusters; ++c)
		{
			std::cout << std::endl << "Convex-Hull " << c << std::endl;
			size_t nPoints = myHACD->GetNPointsCH(c);
			size_t nTriangles = myHACD->GetNTrianglesCH(c);
			HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
			HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
			myHACD->GetCH(c, pointsCH, trianglesCH);
			std::cout << "Points " << nPoints << std::endl;
			
			*nOutPoints[c] = static_cast<int>(nPoints);
			*nOutTriangles[c] = static_cast<int>(nTriangles);
			*pointsOut[c] = (double**)::CoTaskMemAlloc(sizeof(double*) * nTriangles);
			*triangleOut[c] = (long**)::CoTaskMemAlloc(sizeof(long*) * nTriangles);

			for (size_t v = 0; v < nPoints; ++v)
			{
				*pointsOut[c][v] = (double*)::CoTaskMemAlloc(sizeof(double) * 3);

				*pointsOut[c][v][0] = pointsCH[v].X();
				*pointsOut[c][v][1] = pointsCH[v].Y();
				*pointsOut[c][v][2] = pointsCH[v].Z();

				std::cout << v << "\t"
					<< pointsCH[v].X() << "\t"
					<< pointsCH[v].Y() << "\t"
					<< pointsCH[v].Z() << std::endl;
			}

			std::cout << "Triangles " << nTriangles << std::endl;
			for (size_t f = 0; f < nTriangles; ++f)
			{
				*triangleOut[c][f] = (long*)::CoTaskMemAlloc(sizeof(long) * 3);

				*triangleOut[c][f][0] = trianglesCH[f].X();
				*triangleOut[c][f][1] = trianglesCH[f].Y();
				*triangleOut[c][f][2] = trianglesCH[f].Z();

				std::cout << f << "\t"
					<< trianglesCH[f].X() << "\t"
					<< trianglesCH[f].Y() << "\t"
					<< trianglesCH[f].Z() << std::endl;
			}
			delete[] pointsCH;
			delete[] trianglesCH;
		}

		const HACD::Vec3<HACD::Real> * const decimatedPoints = myHACD->GetDecimatedPoints();
		const HACD::Vec3<long> * const decimatedTriangles = myHACD->GetDecimatedTriangles();
		
		bool exportSepFiles = false;
		if (exportSepFiles)
		{
			char outFileName[1024];
			for (size_t c = 0; c < nClusters; c++)
			{
				sprintf_s(outFileName, "%s%s%s_hacd_%lu.wrl", folder.c_str(), PATH_SEP, file.substr(0, file.find_last_of(".")).c_str(), static_cast<unsigned long>(c));
				myHACD->Save(outFileName, false, c);
			}
		}

		HACD::DestroyHACD(myHACD);
		HACD::releaseHeapManager(heapManager);
	}


	__declspec(dllexport) bool ExtractOFFData(
		const char *fileName,
		double*** points,
		int* nPoints, 
		long*** triangles,
		int* nTriangles,
		bool invert)
	{
		std::string fileNameInternal(fileName);

		return LoadOFF(fileNameInternal, *points, nPoints, *triangles, nTriangles, invert);
	}
}

