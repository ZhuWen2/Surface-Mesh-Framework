#include "surfacemeshprocessing.h"
#include <QtWidgets/QApplication>
//#pragma comment(lib,"D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\build\\x64\\Release\\sanjiaohuadll.lib")
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	SurfaceMeshProcessing mainWin;
	/*mainWin.setGeometry(100,100,mainWin.sizeHint().width(),mainWin.sizeHint().height());
	mainWin.resize( mainWin.sizeHint() );*/
	mainWin.showMaximized();

	if( argc > 1 )
	{
		mainWin.open_mesh_from_main(argv[1]);
	}

	return app.exec();
}
