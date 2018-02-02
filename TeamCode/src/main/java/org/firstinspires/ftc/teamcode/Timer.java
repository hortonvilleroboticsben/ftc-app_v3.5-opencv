package org.firstinspires.ftc.teamcode;

public class Timer {

    private long startTime = 0;
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
            reset();
            isWaiting = true;
        }else{
            if(retVal = getElapsedTime() >= time){
                reset();
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