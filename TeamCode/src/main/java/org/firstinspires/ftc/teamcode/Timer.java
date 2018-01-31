package org.firstinspires.ftc.teamcode;

public class Timer {

    private long startTime = 0;
    private Timer t = null;
    private boolean isWaiting = false;

    public Timer(){
        startTime = System.nanoTime();
    }

    public long getElapsedTime(){
        return (System.nanoTime() - startTime)/1000000;
    }

    public void reset(){
        startTime = System.nanoTime();
    }

    public boolean hasWaitFinished(long time){
        boolean retVal = false;
        if(!isWaiting){
            t = new Timer();
            isWaiting = true;
        }else{
            if(retVal = t.getElapsedTime() >= time){
                t.reset();
                isWaiting = false;
            }else isWaiting = true;
        }
        return retVal;
    }

    public boolean isWaiting(){return isWaiting;}

    public String toString(){
        return "ElapsedTime:"+getElapsedTime();
    }
}