#include <Eigen/Dense>
#include <iostream>
#include <boost/python.hpp>
#include <pygtk/pygtk.h>
#include <gtkmm.h>

#include <codrha_local_planner/trajectory.h>
#include <codrha_local_planner/monocycle_flatoutput.h>
#include <string>

using namespace codrha_local_planner;

// using namespace boost::python;
// typedef namespace boost::python bpy;
#define bpy boost::python

const unsigned MonocycleFlatoutput::flatDim;        // 2D curve
const unsigned MonocycleFlatoutput::poseDim;        // unicycle configuration dimension
const unsigned MonocycleFlatoutput::positionDim;    // position in the 2D plane dimension
const unsigned MonocycleFlatoutput::oriDim;         // position in the 2D plane dimension
const unsigned MonocycleFlatoutput::veloDim;        // velocity dimension in 2D plane
const unsigned MonocycleFlatoutput::accelDim;       // acceleration dimension in 2D plane
const unsigned MonocycleFlatoutput::derivOrdForPosition;
const unsigned MonocycleFlatoutput::derivOrdForPose;
const unsigned MonocycleFlatoutput::derivOrdForVelo;
const unsigned MonocycleFlatoutput::derivOrdForAccel;
const unsigned MonocycleFlatoutput::linSpeedIdx;
const unsigned MonocycleFlatoutput::angSpeedIdx;
const unsigned MonocycleFlatoutput::linAccelIdx;
const unsigned MonocycleFlatoutput::angAccelIdx;
const unsigned MonocycleFlatoutput::positionIdx;
const unsigned MonocycleFlatoutput::oriIdx;

unsigned counter = 0;

struct twoDLine
{
        bpy::object x;
        bpy::object y;
};

void plot(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & points, Trajectory<double, MonocycleFlatoutput::flatDim, MonocycleFlatoutput::derivOrdForAccel+1> & traj)
{
        // Python startup code
        Py_Initialize();
        // PyRun_SimpleString("import signal");
        // PyRun_SimpleString("signal.signal(signal.SIGINT, signal.SIG_DFL)");

        // Get python objects figure and canvas (using GTKAgg as backend)
        bpy::object Figure = bpy::object(bpy::handle<>(PyImport_ImportModule("matplotlib.figure"))).attr("Figure");
        bpy::object FigureCanvas = bpy::object(bpy::handle<>(PyImport_ImportModule("matplotlib.backends.backend_gtkagg"))).attr("FigureCanvasGTKAgg");

        // Instantiate figure and canvas
        bpy::object figure = Figure();


        bpy::dict fig_options;
        fig_options["top"] = 0.9;
        fig_options["bottom"] = 0.1;
        fig_options["hspace"] = 0.7;
        figure.attr("subplots_adjust")(*bpy::tuple(), **fig_options);

        // figure.attr("suptitle")("Derivative Degree: " + std::to_string(derivOrdForAccel));

        bpy::object canvas = FigureCanvas(figure);

        // Create a 3 plots in the figure

        // bpy::object axes["position"] = figure.attr("add_subplot")(311);
        // bpy::object axes["speed"] = figure.attr("add_subplot")(312);
        // bpy::object axes["accel"] = figure.attr("add_subplot")(313);
        std::map< std::string, bpy::object > axes;

        axes["position"] = figure.attr("add_subplot") (321);
        axes["speed"] = figure.attr("add_subplot") (322);
        axes["accel"] = figure.attr("add_subplot") (323);
        axes["jitter"] = figure.attr("add_subplot") (324);
        axes["angspeed"] = figure.attr("add_subplot") (325);
        axes["angaccel"] = figure.attr("add_subplot") (326);

        // Add grids and hold
        // for (std::map< std::string, bpy::object >::iterator axis = axes.begin(); axis != axes.end(); ++axis)
        for (auto axis : axes)
        {
                axis.second.attr("hold") (true);
                axis.second.attr("grid") (true);
        }

        // Get parametric variable interval and number of control points
        double parVarInterv = traj.getParVarInterval();
        unsigned nCtrlPts = traj.nParam()/2;
        std::cout << "nCtrlPts " << nCtrlPts << std::endl;

        twoDLine origPts;
        twoDLine interpolation;
        twoDLine interpolation2;
        twoDLine ctrlPts;

        // Create python lists
        origPts.x = bpy::object(bpy::handle<>(PyList_New(0)));
        origPts.y = bpy::object(bpy::handle<>(PyList_New(0)));
        interpolation.x = bpy::object(bpy::handle<>(PyList_New(0)));
        interpolation.y = bpy::object(bpy::handle<>(PyList_New(0)));
        ctrlPts.x = bpy::object(bpy::handle<>(PyList_New(0)));
        ctrlPts.y = bpy::object(bpy::handle<>(PyList_New(0)));
        bpy::object time = bpy::object(bpy::handle<>(PyList_New(0)));
        bpy::object xVelo = bpy::object(bpy::handle<>(PyList_New(0)));
        bpy::object xAccel = bpy::object(bpy::handle<>(PyList_New(0)));
        bpy::object xJitter = bpy::object(bpy::handle<>(PyList_New(0)));

        bpy::list angvel;
        bpy::list angaccel;

        // Store original points in python lists
        for (auto i=0; i < nCtrlPts; ++i)
        {
                origPts.x.attr("append") (points(0, i));
                origPts.y.attr("append") (points(1, i));
        }

        // Compute interpolation line, speeds and times
        for (auto i=0; i < 1000; ++i)
        {
                double evalValue = double(i)/(1000-1)*parVarInterv;

                Eigen::Matrix<double, Eigen::Dynamic, 4> point(traj(evalValue, 3)); // trajectory at evalValue up to the second derivative

                // Eigen::Matrix<double, Eigen::Dynamic, 1> position(traj(evalValue)); // only position

                interpolation.x.attr("append") (point(0, 0));
                interpolation.y.attr("append") (point(1, 0));

                // interpolation2.x.attr("append")(position(0));
                // interpolation2.y.attr("append")(position(1));
                //
                Eigen::Matrix<double, 2, 1> vel = MonocycleFlatoutput::flatToVelocity(point);
                Eigen::Matrix<double, 2, 1> accel = MonocycleFlatoutput::flatToAcceleration(point);

                xVelo.attr("append") (vel(0));
                // xVelo.attr("append") (std::sqrt(point(0,1)*point(0,1) + point(1,1)*point(1,1)));
                xAccel.attr("append") (accel(0));
                // xAccel.attr("append") (std::sqrt(point(0,2)*point(0,2) + point(1,2)*point(1,2)));

                xJitter.attr("append") (point(0,3));

                angvel.append(vel(1));
                angaccel.append(accel(1));

                time.attr("append") (evalValue);
        }

        // Get control points
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> optPar(traj.getCtrlPts());

        for (auto i=0; i < nCtrlPts; ++i)
        {
                ctrlPts.x.attr("append") (optPar(0,i));
                ctrlPts.y.attr("append") (optPar(1,i));
        }

        bpy::list arguments;
        bpy::dict options;

        arguments.append(origPts.x);
        arguments.append(origPts.y);
        arguments.append("ro");
        options["label"] = "original points";
        axes["position"].attr("plot") (*bpy::tuple(arguments), **options);
        axes["position"].attr("set_xlabel")("x");
        axes["position"].attr("set_ylabel")("y");

        arguments = bpy::list();
        arguments.append(ctrlPts.x);
        arguments.append(ctrlPts.y);
        arguments.append("g*");
        options["label"] = "control points";
        axes["position"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(interpolation.x);
        arguments.append(interpolation.y);
        options["label"] = "interpolation";
        axes["position"].attr("plot") (*bpy::tuple(arguments), **options);

        axes["angspeed"].attr("plot") (time, angvel);
        axes["angspeed"].attr("set_title") ("Angular vel");
        axes["angaccel"].attr("plot") (time, angaccel);
        axes["angaccel"].attr("set_title") ("Angular accel");

        bpy::dict leg_options;
        leg_options["fontsize"] = 8;
        // axes["position"].attr("legend")(**leg_options);
        axes["position"].attr("legend")(*bpy::tuple(), **leg_options);
        // axes["position"].attr("legend")();

        axes["position"].attr("set_title") ("Path xy");

        // axes["position"].attr("plot")(interpolation2.x, interpolation2.y, "k");
        axes["speed"].attr("plot") (time, xVelo);
        axes["speed"].attr("set_title") ("Linear vel");
        axes["speed"].attr("set_xlabel")("time");

        axes["accel"].attr("plot") (time, xAccel);
        axes["accel"].attr("set_title") ("Linear accel");
        axes["accel"].attr("set_xlabel")("time");

        axes["jitter"].attr("plot") (time, xJitter);
        axes["jitter"].attr("set_title") ("Jitter along x");
        axes["jitter"].attr("set_xlabel")("time");

        // axes["position"].attr("get_yaxis")().attr("get_major_formatter")().attr("set_useOffset")(false);
        // axes["speed"].attr("get_yaxis")().attr("get_major_formatter")().attr("set_useOffset")(false);
        // axes["accel"].attr("get_yaxis")().attr("get_major_formatter")().attr("set_useOffset")(false);

        // plt_tight_layout();
        // tight_layout.attr("__call__")();

        canvas.attr("draw")();

        bpy::dict savefig_options;
        savefig_options["bbox_inches"] = "tight";
        savefig_options["dpi"] = 300;

        bpy::list savefig_arguments1;
        std::stringstream ss;
        ss << "test_trajctory_fig" << std::to_string(counter) << ".png";
        // savefig_options = bpy::dict();
        savefig_arguments1.append(ss.str());
        figure.attr("savefig") (*bpy::tuple(savefig_arguments1), **savefig_options);

        bpy::list savefig_arguments2;
        ss.clear(); //clear any bits set
        ss.str(std::string());
        ss << "test_trajctory_fig" << std::to_string(counter) << ".pdf";
        savefig_arguments2.append(ss.str());
        figure.attr("savefig") (*bpy::tuple(savefig_arguments2), **savefig_options);

        counter++;
        // GTK stuff
        //
        // Normal Gtk startup code
        Gtk::Main kit(0,0);

        // Create our window
        Gtk::Window window;
        window.set_title("Figure 1");
        window.set_default_size(1000, 600);

        // Grab the Gtk::DrawingArea from the canvas.
        Gtk::DrawingArea *graphic = Glib::wrap(GTK_DRAWING_AREA(pygobject_get(canvas.ptr())));

        // Add the graphic to the window.
        window.add(*graphic);
        window.show_all();

        // And start the Gtk event loop.
        Gtk::Main::run(window);
}

int main()
{

        // Constants
        // const unsigned nIntervNonNull = 5;
        const unsigned nCtrlPts = 8;

        // const unsigned splDeg can be = derivOrdForAccel;
        const unsigned nIntervNonNull = nCtrlPts - MonocycleFlatoutput::derivOrdForAccel;
        //
        const double parVarInterv = 10.; // Parametric variable (usualy time) max value (0.0 is always the min)

        // Create a trajectory
        Trajectory<double, MonocycleFlatoutput::flatDim, MonocycleFlatoutput::derivOrdForAccel+1> traj(nCtrlPts); // MonocycleFlatoutput::flatDim, derivOrdForAccel, nInterNonNull
        std::srand((unsigned) time(0));
        Eigen::Matrix<double, MonocycleFlatoutput::flatDim, Eigen::Dynamic> points (MonocycleFlatoutput::flatDim, nCtrlPts);
        // for (unsigned i = 0; i < MonocycleFlatoutput::flatDim; ++i)
                // points.row(i) = Eigen::RowVectorXd::Random(nCtrlPts);
        // std::cout << points << std::endl;
        points.row(0) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 5.0);
        points.row(1) = 0.1*Eigen::RowVectorXd::Random(nCtrlPts);
        // points.row(1) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 2.0).array().sin();

        // Interpolate points
        std::cout << points.cols() << std::endl;
        traj.interpolate(points, parVarInterv);

        plot(points, traj);

        // just test the effect of changing the parametric variable (time)
        // for (auto j = 0; j < 1; ++j)
        // {
        // points beign used as control points
        // traj.update(traj.getCtrlPts(), 50);

        // Interpolate points
        // plot(points, traj);
        // }


        // Create nCtrlPts 2D points equaly spaced from 0, 0 to 1, 1
        // for (auto j = 0; j < 1; ++j)
        // {
        // std::srand((unsigned) time(0));
        // Eigen::Matrix<double, MonocycleFlatoutput::flatDim, Eigen::Dynamic> points (dim, nCtrlPts);
        // for (unsigned i = 0; i < MonocycleFlatoutput::flatDim; ++i)
        //         points.row(i) = Eigen::RowVectorXd::Random(nCtrlPts);
        // points.row(i) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 5.0);

        // points beign used as control points
        // traj.update(traj.getCtrlPts(), 50);

        // Interpolate points
        // plot(points, traj);
        // }

        return 0;
}
