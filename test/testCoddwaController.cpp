// #include <Eigen/Core>
// #include <codrha_local_planner/codrha_planner.h>
#include <codrha_local_planner/codrha_controller.h>
#include <codrha_local_planner/trajectory.h>
#include <codrha_local_planner/monocycle_flatoutput.h>
#include <iostream>
#include <limits>

#include <boost/python.hpp>
#include <pygtk/pygtk.h>
#include <gtkmm.h>

#define bpy boost::python

using namespace codrha_local_planner;
typedef MonocycleFlatoutput MF;

struct twoDLine
{
        bpy::object x;
        bpy::object y;
};
unsigned counter = 0;

void plot(bpy::list tim, bpy::list lv, bpy::list av, bpy::list lvc,
          bpy::list avc, bpy::list lvr, bpy::list avr,
          bpy::list x, bpy::list y, bpy::list xp, bpy::list yp);

// typedef codrha_local_planner clp;
int main(int argc, char *argv[]) {
        // CODRHAPlanner planner("planner");
        Controller control("controller");

        double max_lin_vel, max_ang_vel, prediction_time,
               integration_time_step;

        double aux[] = {0.04200441, 0.27468742, -0.01248822, 1.00119437,
                        0.00545974, 1.03107639};
        std::vector<double> dy_mod(aux, aux + sizeof(aux) / sizeof(double));

        int nCtrlPts = 8;
        double parVarInterv = 2.;
        double time_step = 0.0001;

        if (argc < 3)
        {
                std::cout << "You forgot my precious arguments! Give me a controller prediction time and a evaluation time, pls! Bye XO" << std::endl;
                return 0;
        }
        else if (argc == 3)
        {
                prediction_time = atof(argv[1]);
                integration_time_step = atof(argv[2]);
                max_lin_vel = std::numeric_limits<double>::infinity();
                max_ang_vel = std::numeric_limits<double>::infinity();
        }
        else if (argc == 5)
        {
                prediction_time = atof(argv[1]);
                integration_time_step = atof(argv[2]);
                max_lin_vel = atof(argv[3]);
                max_ang_vel = atof(argv[4]);
        }
        else
        {
                std::cout << "You forgot my precious arguments! Give me a controller prediction time and a evaluation time, pls! Bye XO" << std::endl;
                return 0;
        }

        control.reconfigure(prediction_time, max_lin_vel, max_ang_vel,
                            integration_time_step, dy_mod);

        // Create points
        std::srand((unsigned) time(0));
        Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> points (MF::flatDim, nCtrlPts);
        // for (unsigned i = 0; i < MF::flatDim; ++i)
        points.row(0) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 1.25);
        // points.row(1) = .0001*Eigen::RowVectorXd::Random(nCtrlPts);
        // points(1, 4) = 0.0;
        // points(1, 5) = 0.0;
        // points(1, 6) = 0.0;
        // points(1, 7) = 0.0;
        // points.row(1).setZero();
        // points.row(1) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 2.0);
        points.row(1) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 1.25).array().pow(2.);

        // Create trajectory
        Trajectory<double, MF::flatDim, MF::derivOrdForAccel> ref_trajectory(nCtrlPts);
        ref_trajectory.interpolate(points, parVarInterv);

        MF::MatForAccelD derivFlat(ref_trajectory(0.0, MF::derivOrdForAccel));
        MF::PoseVectorD ref_pose(MF::flatToPose(derivFlat));
        MF::VeloVectorD ref_vel(MF::flatToVelocity(derivFlat));
        MF::AccelVectorD ref_acc(MF::flatToAcceleration(derivFlat));

        std::cout << ref_pose << std::endl;
        std::cout << ref_vel << std::endl;
        std::cout << ref_acc << std::endl;

        Eigen::Matrix<double, MF::positionDim, MF::positionDim> rot;
        rot << cos(MF::yawInPose(ref_pose)(0,0)), sin(MF::yawInPose(ref_pose)(0,0)), -sin(MF::yawInPose(ref_pose)(0,0)), cos(MF::yawInPose(ref_pose)(0,0));

        // ref_trajectory.update(rot*ref_trajectory.getCtrlPts());
        ref_trajectory.update(rot*(ref_trajectory.getCtrlPts() - MF::positionInPose(ref_pose).replicate(1, nCtrlPts)));

        derivFlat = ref_trajectory(0.0, MF::derivOrdForAccel);
        ref_pose = MF::flatToPose(derivFlat);
        ref_vel = MF::flatToVelocity(derivFlat);
        ref_acc = MF::flatToAcceleration(derivFlat);

        std::cout << ref_pose << std::endl;
        std::cout << ref_vel << std::endl;
        std::cout << ref_acc << std::endl;

        // std::cin.get();

        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double u = ref_vel(0);
        double w = ref_vel(1);

        unsigned plan_stage = 1;
        double lin_vel_output;
        double ang_vel_output;
        bpy::list time_vec;
        bpy::list lin_vel;
        bpy::list ang_vel;
        bpy::list lin_vel_ctlr;
        bpy::list ang_vel_ctlr;
        bpy::list lin_vel_re;
        bpy::list ang_vel_re;
        bpy::list x_re;
        bpy::list y_re;
        bpy::list x_pl;
        bpy::list y_pl;
        for (double eval_time = 0.0; eval_time < parVarInterv - prediction_time; eval_time += time_step)
        {
                derivFlat = ref_trajectory(eval_time, MF::derivOrdForAccel);
                ref_pose = MF::flatToPose(derivFlat);
                ref_vel = MF::flatToVelocity(derivFlat);
                ref_acc = MF::flatToAcceleration(derivFlat);

                control.computeVelocityCommands(
                        x, y, theta, u, w, ref_trajectory,
                        eval_time, parVarInterv, plan_stage,
                        lin_vel_output, ang_vel_output);

                double lin_acc_ref = ref_acc[0];
                double ang_acc_ref = ref_acc[1];
                double lin_vel_ref = ref_vel[0];
                double ang_vel_ref = ref_vel[1];
                double x_ref = ref_pose[0];
                double y_ref = ref_pose[1];
                double theta_ref = ref_pose[2];

                // double x_feedback, double y_feedback, double psi_feedback,
                // double v_feedback, double omega_feedback,
                // const Trajectory<double, MonocycleFlatoutput::flatDim, MonocycleFlatoutput::derivOrdForAccel> & trajectory,
                // const Eigen::Matrix<double, MonocycleFlatoutput::poseDim, 1> trajectory_transform,
                // double eval_time, double plan_time, unsigned plan_stage,
                // double & lin_vel_output, double & ang_vel_output

                // SAVE
                lin_vel.append(lin_vel_ref);
                ang_vel.append(ang_vel_ref);
                lin_vel_ctlr.append(lin_vel_output);
                ang_vel_ctlr.append(ang_vel_output);
                time_vec.append(eval_time);
                lin_vel_re.append(u);
                ang_vel_re.append(w);
                x_re.append(x);
                y_re.append(y);
                x_pl.append(x_ref);
                y_pl.append(y_ref);

                // Update physics CONTROLLER
                x = u*cos(theta)*time_step + x;
                y = u*sin(theta)*time_step + y;
                w*time_step + theta;
                theta = common::wrapScalarToPi(w*time_step + theta);
                double aux = (dy_mod[2]/dy_mod[1]*w*w - dy_mod[3]/dy_mod[0]*u + lin_vel_output/dy_mod[0])*time_step + u;
                w = (-dy_mod[4]/dy_mod[1]*u*w - dy_mod[5]/dy_mod[1]*w + ang_vel_output/dy_mod[1])*time_step + w;
                u = aux;

                // Update physics REF
                // x = u*cos(theta)*time_step + x;
                // y = u*sin(theta)*time_step + y;
                // w*time_step + theta;
                // theta = common::wrapScalarToPi(w*time_step + theta);
                // double aux = (dy_mod[2]/dy_mod[1]*w*w - dy_mod[3]/dy_mod[0]*u + lin_vel_ref/dy_mod[0])*time_step + u;
                // w = (-dy_mod[4]/dy_mod[1]*u*w - dy_mod[5]/dy_mod[1]*w + ang_vel_ref/dy_mod[1])*time_step + w;
                // u = aux;

                // std::cout << lin_vel_ref << std::endl;
                // std::cout << ang_vel_ref << std::endl;
                // std::cout << lin_vel_output << std::endl;
                // std::cout << ang_vel_output << std::endl;
                // std::cout << "ref_pose:\n" << ref_pose << std::endl;
        }

         // PLOT
         plot(time_vec, lin_vel, ang_vel, lin_vel_ctlr, ang_vel_ctlr,
              lin_vel_re, ang_vel_re, x_re, y_re, x_pl, y_pl);

        std::cout << "All done" << std::endl;
        return 0;
}

void plot(bpy::list tim, bpy::list lv, bpy::list av, bpy::list lvc,
          bpy::list avc, bpy::list lvr, bpy::list avr,
          bpy::list x, bpy::list y, bpy::list xp, bpy::list yp)
{
        Py_Initialize();

        // Get python objects figure and canvas (using GTKAgg as backend)
        bpy::object Figure = bpy::object(bpy::handle<>(PyImport_ImportModule("matplotlib.figure"))).attr("Figure");
        // bpy::object FigureCanvas = bpy::object(bpy::handle<>(PyImport_ImportModule("matplotlib.backends.backend_gtkagg"))).attr("FigureCanvasGTKAgg");
        bpy::object FigureCanvas = bpy::object(bpy::handle<>(PyImport_ImportModule("matplotlib.backend_bases"))).attr("FigureCanvasBase");

        // Instantiate figure and canvas
        bpy::object figure = Figure();

        // Ajust subplot configuration
        bpy::dict fig_options;
        fig_options["top"] = 0.9;
        fig_options["bottom"] = 0.1;
        fig_options["hspace"] = 0.4;
        fig_options["wspace"] = 0.4;
        figure.attr("subplots_adjust")(*bpy::tuple(), **fig_options);

        bpy::object canvas = FigureCanvas(figure);

        std::map< std::string, bpy::object > axes;

        axes["path"] = figure.attr("add_subplot") (211);
        axes["lin"] = figure.attr("add_subplot") (223);
        axes["ang"] = figure.attr("add_subplot") (224);

        for (auto axis : axes)
        {
                axis.second.attr("hold") (true);
                axis.second.attr("grid") (true);
        }

        bpy::list arguments;
        bpy::dict options;

        // arguments = bpy::list();
        arguments.append(tim);
        arguments.append(lv);
        arguments.append("b");
        options["label"] = "planner";
        axes["lin"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(tim);
        arguments.append(lvc);
        arguments.append("g");
        options["label"] = "controller";
        axes["lin"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(tim);
        arguments.append(lvr);
        arguments.append("r");
        options["label"] = "real";
        axes["lin"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(tim);
        arguments.append(av);
        arguments.append("b");
        options["label"] = "planner";
        axes["ang"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(tim);
        arguments.append(avc);
        arguments.append("g");
        options["label"] = "controller";
        axes["ang"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(tim);
        arguments.append(avr);
        arguments.append("r");
        options["label"] = "real";
        axes["ang"].attr("plot") (*bpy::tuple(arguments), **options);

        bpy::dict leg_options;
        leg_options["fontsize"] = 8;
        // axes["position"].attr("legend")(**leg_options);
        axes["lin"].attr("legend")(*bpy::tuple(), **leg_options);
        axes["ang"].attr("legend")(*bpy::tuple(), **leg_options);

        axes["lin"].attr("set_title") ("Lienar Velocity");
        axes["lin"].attr("set_ylabel")("m/s");
        axes["lin"].attr("set_xlabel")("time (s)");

        axes["ang"].attr("set_title") ("Angular Velocity");
        axes["ang"].attr("set_ylabel")("rad/s");
        axes["ang"].attr("set_xlabel")("time (s)");

        arguments = bpy::list();
        arguments.append(x);
        arguments.append(y);
        arguments.append("r");
        options["label"] = "real";
        axes["path"].attr("plot") (*bpy::tuple(arguments), **options);

        arguments = bpy::list();
        arguments.append(xp);
        arguments.append(yp);
        arguments.append("b");
        options["label"] = "planner";
        axes["path"].attr("plot") (*bpy::tuple(arguments), **options);
        axes["path"].attr("set_title") ("Path xy");
        axes["path"].attr("set_xlabel")("x");
        axes["path"].attr("set_ylabel")("y");

        // DRAW
        canvas.attr("draw")();
        bpy::dict savefig_options;
        savefig_options["bbox_inches"] = "tight";
        savefig_options["dpi"] = 300;

        bpy::list savefig_arguments1;
        std::stringstream ss;
        ss << "./test_controller_fig" << std::to_string(counter) << ".png";
        // savefig_options = bpy::dict();
        savefig_arguments1.append(ss.str());
        figure.attr("savefig") (*bpy::tuple(savefig_arguments1), **savefig_options);

        bpy::list savefig_arguments2;
        ss.clear(); //clear any bits set
        ss.str(std::string());
        ss << "./test_controller_fig" << std::to_string(counter) << ".pdf";
        savefig_arguments2.append(ss.str());
        figure.attr("savefig") (*bpy::tuple(savefig_arguments2), **savefig_options);


        counter++;

        // GTK stuff
        //
        // Normal Gtk startup code
        // Gtk::Main kit(0,0);
        //
        // Create our window
        // Gtk::Window window;
        // window.set_title("Figure 1");
        // window.set_default_size(1000, 600);
        //
        // Grab the Gtk::DrawingArea from the canvas.
        // Gtk::DrawingArea *graphic = Glib::wrap(GTK_DRAWING_AREA(pygobject_get(canvas.ptr())));
        //
        // Add the graphic to the window.
        // window.add(*graphic);
        // window.show_all();
        //
        // And start the Gtk event loop.
        // Gtk::Main::run(window);

}
