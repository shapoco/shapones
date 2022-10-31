#ifndef FC_SCREEN_HPP
#define FC_SCREEN_HPP

#include <wx/wx.h>

class FcScreen : public wxScrolledWindow {
public:
    FcScreen(wxFrame *parent, wxWindowID id);
    void OnPaint(wxPaintEvent &event);
    void OnKeyDown(wxKeyEvent &event);
    void OnKeyUp(wxKeyEvent &event);
    void Render();

private:
    wxFrame *owner;
    wxImage frame_buff;
    DECLARE_EVENT_TABLE();
};


#endif
