
/* Code for Assignment ?? 
 * Name:
 * Usercode:
 * ID:
 */

import ecs100.*;
import java.util.*;
import java.io.*;
import java.awt.*;
import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.font.FontRenderContext;
import java.awt.font.TextLayout;
import java.awt.geom.AffineTransform;
import java.awt.geom.FlatteningPathIterator;
import java.awt.geom.PathIterator;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;


/** <description of class Main>
 */
public class Main{

    private Arm arm;
    private Drawing drawing;
    private ToolPath tool_path;
    // state of the GUI
    private int state; // 0 - nothing
    // 1 - inverse point kinematics - point
    // 2 - enter path. Each click adds point  
    // 3 - enter path pause. Click does not add the point to the path
    // 4 - circle
    /**      */
    public Main(){
        UI.initialise();
        UI.addButton("xy to angles", this::inverse);
        UI.addButton("Enter path XY", this::enter_path_xy);
        UI.addButton("Save path XY", this::save_xy);
        UI.addButton("Load path XY", this::load_xy);
        UI.addButton("Save path Ang", this::save_ang);
        UI.addButton("Load path Ang:Play", this::load_ang);
        UI.addButton("Circle", this::addCircle);
        UI.addButton("rect", this::addRect);
        UI.addButton("SendtoPi", this::sendToPi);
        UI.addButton("Skynet", this::drawSky);
        // UI.addButton("Quit", UI::quit);
        UI.setMouseMotionListener(this::doMouse);
        UI.setKeyListener(this::doKeys);

        //ServerSocket serverSocket = new ServerSocket(22); 
        this.tool_path = new ToolPath();
        this.arm = new Arm();
        this.drawing = new Drawing();
        this.run();
        arm.draw();
    }

    public void addCircle(){
        state = 4;
    }

    public void addRect(){
        state=5;
    }
   
    public void doKeys(String action){
        UI.printf("Key :%s \n", action);
        if (action.equals("b")) {
            // break - stop entering the lines
            state = 3;
            //

        }

    }

    public void drawSky(){
        Font font = new JButton().getFont();
        BufferedImage bt = new BufferedImage(2,2,BufferedImage.TYPE_3BYTE_BGR);
        FontRenderContext frc = bt.getGraphics().getFontMetrics(font).getFontRenderContext();
        TextLayout layout = new TextLayout("Skynet",font,frc);
        FlatteningPathIterator it = new FlatteningPathIterator(layout.getOutline(null).getPathIterator(AffineTransform.getScaleInstance(5,5)),100);
        double[] points = new double[6];
        while (!it.isDone()) {
            int i = it.currentSegment(points);
            if(i == PathIterator.SEG_MOVETO)
            drawing.add_point_to_path(200 + points[0], 200 +points[1],i != PathIterator.SEG_MOVETO);
            it.next();
        }
        drawing.draw();
    }
    
    public void sendToPi() {
        try {
            ProcessBuilder pb = new ProcessBuilder("script", "test", "scp line.txt pi@10.140.66.166:/home/pi/Arm/");
            Process p = pb.start();
            InputStream stream = p.getInputStream();
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(p.getOutputStream()));
            Scanner s = new Scanner(stream);
            while (p.isAlive()) {
                String str = s.next();
                UI.println(str);

                if (str.contains("password"))
                    writer.write("pi\n");
                writer.flush();
            }

        }catch(Exception e){
            UI.println(e);
        }
    }
    
    public void doMouse(String action, double x, double y) {
        //UI.printf("Mouse Click:%s, state:%d  x:%3.1f  y:%3.1f\n",
        //   action,state,x,y);
        UI.clearGraphics();
        String out_str=String.format("%3.1f %3.1f",x,y);
        UI.drawString(out_str, x+10,y+10);
        // 
        if ((state == 4)&&(action.equals("clicked"))){
            double x1 = x;
            double y1 = y;
            double r = 50;
            for(int t = 0; t<=360;t=t+4){
                double cx = x1 + r*Math.cos(t*Math.PI/180);
                double cy = y1 + r*Math.sin(t*Math.PI/180);
                UI.printf("Adding point x=%f y=%f\n",cx,cy);
                drawing.add_point_to_path(cx,cy,true); // add point with pen down

                arm.inverseKinematic(cx,cy);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
            }
        }
        if ((state == 5)&&(action.equals("clicked"))){
                drawing.add_point_to_path(x,y,true); // add point with pen down
                arm.inverseKinematic(x,y);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
                
                drawing.add_point_to_path(x+50,y,true); // add point with pen down
                arm.inverseKinematic(x +50,y);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
                
                drawing.add_point_to_path(x+50,y-50,true); // add point with pen down
                arm.inverseKinematic(x +50,y-50);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
                
                drawing.add_point_to_path(x,y-50,true); // add point with pen down
                arm.inverseKinematic(x,y-50);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
                
                drawing.add_point_to_path(x,y,true); // add point with pen down
                arm.inverseKinematic(x,y);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
        }
        if  ((state == 6)&&(action.equals("clicked"))){
            double r = 25;
            for(int t = 0; t<=360;t=t+8){
                double cx = x + r*Math.cos(t*Math.PI/180);
                double cy = y + r*Math.sin(t*Math.PI/180);
                UI.printf("Adding point x=%f y=%f\n",cx,cy);
                drawing.add_point_to_path(cx,cy,true); // add point with pen down

                arm.inverseKinematic(cx,cy);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
            }
            double x1 = x;
            double y1 = y - r*2;
            drawing.add_point_to_path(x1 + r,y1,false);
            for(int t = 0; t<=360;t=t+8){
                double cx = x1 + r*Math.cos(t*Math.PI/180);
                double cy = y1 + r*Math.sin(t*Math.PI/180);
                UI.printf("Adding point x=%f y=%f\n",cx,cy);
                drawing.add_point_to_path(cx,cy,true); // add point with pen down

                arm.inverseKinematic(cx,cy);
                //arm.draw();
                drawing.draw();
                drawing.print_path();
            }
            drawing.add_point_to_path(x,y,false);
            
            int b = 10;
            drawing.add_point_to_path(x,y,true); // add point with pen down
            arm.inverseKinematic(x,y);
            //arm.draw();
            drawing.draw();
            drawing.print_path();
            
            drawing.add_point_to_path(x,y-b,false);
            drawing.add_point_to_path(x,y-b,true); // add point with pen down
            arm.inverseKinematic(x,y-b);
            //arm.draw();
            drawing.draw();
            drawing.print_path();
            
            drawing.add_point_to_path(x,y+b,false);
            drawing.add_point_to_path(x,y+b,true); // add point with pen down
            arm.inverseKinematic(x,y+b);
            //arm.draw();
            drawing.draw();
            drawing.print_path();
            
        }
        
        if ((state == 1)&&(action.equals("clicked"))){
            // draw as 
            arm.inverseKinematic(x,y);
            arm.draw();
            return;
        }

        if ( ((state == 2)||(state == 3))&&action.equals("moved") ){
            // draw arm and path
            arm.inverseKinematic(x,y);
            arm.draw();
            // draw segment from last entered point to current mouse position
            if ((state == 2)&&(drawing.get_path_size()>0)){
                PointXY lp = new PointXY();
                lp = drawing.get_path_last_point();
                //if (lp.get_pen()){
                UI.setColor(Color.GRAY);
                UI.drawLine(lp.get_x(),lp.get_y(),x,y);
                // }
            }
            drawing.draw();
        }

        // add point
        if(arm.inverseKinematic(x,y) == true){
            if (   (state == 2) &&(action.equals("clicked"))){
                // add point(pen down) and draw
                UI.printf("Adding point x=%f y=%f\n",x,y);
                drawing.add_point_to_path(x,y,true); // add point with pen down

                arm.inverseKinematic(x,y);
                arm.draw();
                drawing.draw();
                drawing.print_path();
            }
        }

        if (   (state == 3) &&(action.equals("clicked"))){
            // add point and draw
            //UI.printf("Adding point x=%f y=%f\n",x,y);
            drawing.add_point_to_path(x,y,false); // add point wit pen up

            arm.inverseKinematic(x,y);
            arm.draw();
            drawing.draw();
            drawing.print_path();
            state = 2;
        }

    }

    public void save_xy(){
        state = 0;
        String fname = UIFileChooser.save();
        drawing.save_path(fname);
    }

    public void enter_path_xy(){
        state = 2;
    }

    public void inverse(){
        state = 1;
        arm.draw();
    }

    public void load_xy(){
        state = 0;
        String fname = UIFileChooser.open();
        drawing.load_path(fname);
        drawing.draw();

        arm.draw();
    }

    // save angles into the file
    public void save_ang(){
        state = 0;
        String fname = UIFileChooser.save();
        tool_path.convert_drawing_to_angles(drawing,arm,fname);
        tool_path.convert_angles_to_pwm(arm);
        tool_path.save_pwm_file(fname);
    }

    //loads angles from a file
    public void load_ang(){

    }

    public void run() {
        while(true) {
            arm.draw();
            UI.sleep(20);
        }
    }

    public static void main(String[] args){
        Main obj = new Main();
    }    

}
