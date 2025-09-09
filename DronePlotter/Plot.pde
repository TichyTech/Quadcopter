void drawXYZ(long[] ms, float[][] vals, int x, int y, int plot_width, int plot_height, float full_scale, int start_plot, int points, String name){  
  final long ms_window = 2000;
  final float pixel_per_ms = float(plot_width)/ms_window;
  final float pixel_per_val = plot_height/(2*full_scale);
    
  colorMode(HSB, 1);
  fill(0.3);
  stroke(0);
  strokeWeight(2);
  rect(x, y, plot_width, plot_height, 10);  // Rounded rectangle
  
  stroke(1);
  fill(1);
  textAlign(CENTER, CENTER);
  text(name, x + plot_width/2, y - 25);
  
  textAlign(LEFT, CENTER);
  text(str(full_scale), x , y + 20);
  text(str(-full_scale), x , y+plot_height - 20);
  colorMode(RGB, 1);

  int len = ms.length;
  if (len < 2 || points < 2) return;
  long last_ms = ms[(start_plot + points - 1) % len];
  long first_ms = last_ms - ms_window;
  
  for (int inc = 1; inc < points; inc++){
    int i = (start_plot + inc) % len;
    int prev_i = (start_plot + inc - 1) % len;
    long cur_ms = ms[i];
    if (cur_ms < first_ms) continue;
    int start_x = int((ms[prev_i] - first_ms)*pixel_per_ms);
    int end_x = int((cur_ms - first_ms)*pixel_per_ms);
    for (int dim = 0 ; dim < 3; dim++){
       colorMode(HSB, 1);
       stroke(float(dim)/3, 1, 1);
       int start_y = int(vals[prev_i][dim] * pixel_per_val);
       int end_y = int(vals[i][dim] * pixel_per_val);
       line(x + start_x, y - start_y + plot_height/2, x + end_x, y - end_y + plot_height/2);
    }    
  }
  
  colorMode(RGB, 1);
  
  int total_samples = 0;
  float[] max = {-full_scale, -full_scale, -full_scale};
  float[] min = {full_scale, full_scale, full_scale};
  float[] mean = {0,0,0};
  for (int inc = 1; inc < points; inc++){
    int i = (start_plot + inc) % len;
    long cur_ms = ms[i];
    if (cur_ms < first_ms) continue;
    for (int axis=0; axis<3; axis++){
      float val = vals[i][axis];
      max[axis] = val > max[axis] ? val : max[axis];
      min[axis] = val < min[axis] ? val : min[axis];
      mean[axis] += val;
    }
    total_samples += 1;
  }
  
  mean[0] = mean[0] / float(total_samples);
  mean[1] = mean[1] / float(total_samples);
  mean[2] = mean[2] / float(total_samples);

  drawVector3(min, x, y-160, "-", 1);
  drawVector3(max, x+160, y-160, "+", 1);
  drawVector3(mean, x+320, y-160, "=", 1);

}

void plot_diff(float[] diff, int x, int y, String title){
  plot_diff_XY(diff, x, y, title);
  plot_diff_Z(diff, x + 300, y);

}

void plot_diff_XY(float[] diff, int x, int y, String title){
  int hh = 300;
  int ww = 300;
  float scale = 50;
  
  colorMode(HSB, 1);
  fill(0.9);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle
  
  colorMode(RGB, 1);
  fill(1, 1, 1);
  stroke(0);
  strokeWeight(2);
  circle(x + ww/2, y + hh/2, 4*scale);
  circle(x + ww/2, y + hh/2, 2*scale);
  strokeWeight(1);
  line(x + ww/2, y, x + ww/2, y+hh);
  line(x, y + hh/2, x + ww, y + hh/2);
  
  stroke(0);
  fill(0);
  textAlign(RIGHT, TOP);
  text(title, x + ww, y);
  textAlign(LEFT, CENTER);
  text("W", x, y+hh/2);
  textAlign(CENTER, TOP);
  text("N", x + ww/2, y);

  colorMode(RGB, 1);
  fill(1, 0, 0);
  stroke(0);
  strokeWeight(2);
  circle(x + ww/2 - int(diff[1] * scale), y + hh/2 - int(diff[0] * scale), 15);
  
}

void plot_diff_Z(float[] diff, int x, int y){
  int hh = 300;
  int ww = 50;
  float scale = 50;
  
  colorMode(HSB, 1);
  fill(0.9);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle
  
  colorMode(RGB, 1);
  fill(1, 1, 1);
  stroke(0);
  strokeWeight(2);
  line(x, y+hh/2, x + ww, y+hh/2);
  line(x, y+hh/2-scale, x + ww, y+hh/2-scale);
  line(x, y+hh/2+scale, x + ww, y+hh/2+scale);
  line(x, y+hh/2-2*scale, x + ww, y+hh/2-2*scale);
  line(x, y+hh/2+2*scale, x + ww, y+hh/2+2*scale);
  
  colorMode(RGB, 1);
  fill(1, 0, 0);
  stroke(0);
  strokeWeight(2);
  rect(x, y + hh/2 - 10 - int(diff[2] * scale), ww, 20, 10);
  
}
